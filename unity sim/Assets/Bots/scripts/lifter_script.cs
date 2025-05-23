using UnityEngine;
using System.Linq;
using System.Collections;
using UnityEngine.AI;

public class Lifter : MonoBehaviour
{
    public enum LifterState { WaitingForRover, MovingBaseToWallTarget, PositioningCrane, BrickPlacedCooldown }
    public LifterState currentState = LifterState.WaitingForRover;

    [Header("References")]
    public NavMeshAgent agent;
    public GameObject brickVisualOnCrane; // The visual representation of the brick on the crane arm
    public Transform brickAnchor;        // Empty child of Lifter (e.g., "CraneHook") for brick parenting & local animation

    [Header("Rover Interaction")]
    public Transform roverHandoverPoint; // Where the Rover should arrive to deliver the brick

    [Header("Crane Animation Settings")]
    public Vector3 liftedBrickLocalPosition = new Vector3(0, 0, 0); // Phase 1: Local pos for brick relative to anchor (lifted & ready)
    public Vector3 liftedBrickLocalRotationEuler = new Vector3(0, 0, 0);   // Phase 1: Local rotation for brick relative to anchor
    public float brickAnimationSpeed = 1.0f;
    public float placementCooldown = 2.0f; // Small delay after placing a brick

    [Header("Wall Placement Settings")]
    public float approachOffsetDistance = 1.0f; // How far from the wall target the Lifter base stops

    private Target currentWallTarget;
    private bool isCraneAnimating = false;
    private Coroutine currentAnimationCoroutine;

    void Start()
    {
        if (agent == null) agent = GetComponent<NavMeshAgent>();
        if (agent == null) Debug.LogError("Lifter: NavMeshAgent component not found or assigned!");
        if (brickVisualOnCrane == null) Debug.LogError("Lifter: Brick Visual On Crane is not assigned!");
        if (brickAnchor == null) Debug.LogError("Lifter: Brick Anchor transform is not assigned!");
        if (roverHandoverPoint == null) Debug.LogError("Lifter: Rover Handover Point is not assigned!");

        brickVisualOnCrane.SetActive(false); // Crane starts empty
        agent.updateRotation = false; // For straight movement along the wall, if desired
        Debug.Log("Lifter Online. State: " + currentState);
    }

    void Update()
    {
        switch (currentState)
        {
            case LifterState.WaitingForRover:
                // Logic for this state is mainly passive, waiting for Rover to call SignalBrickDelivery
                break;
            case LifterState.MovingBaseToWallTarget:
                CheckNavArrival();
                break;
            case LifterState.PositioningCrane:
                // Animation is running, wait for it to finish (handled by coroutine)
                break;
            case LifterState.BrickPlacedCooldown:
                // Managed by a simple timer or Invoke in the coroutine's completion
                break;
        }
    }

    void SetNavDestination(Vector3 pos)
    {
        agent.SetDestination(pos);
        agent.isStopped = false;
    }

    void CheckNavArrival()
    {
        if (!isCraneAnimating && !agent.pathPending && agent.hasPath && !agent.isStopped)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                OnNavBaseArrival();
            }
        }
    }

    void OnNavBaseArrival()
    {
        Debug.Log("Lifter base arrived at wall target approach point.");
        agent.isStopped = true; // Stop base movement
        // agent.ResetPath(); // Optional

        if (currentState == LifterState.MovingBaseToWallTarget && currentWallTarget != null)
        {
            currentState = LifterState.PositioningCrane;
            Debug.Log("Lifter: Starting crane animation to place brick at " + currentWallTarget.name);
            if(currentAnimationCoroutine != null) StopCoroutine(currentAnimationCoroutine);
            currentAnimationCoroutine = StartCoroutine(AnimateCraneToPlaceBrick(currentWallTarget));
        }
    }

    Target FindNextAvailableWallTarget()
    {
        return FindObjectsOfType<Target>()
            .Where(t => t.IsAvailable())
            .OrderBy(t => Vector3.Distance(transform.position, t.transform.position)) // Or some other logic for wall order
            .LastOrDefault();
    }

    // Called by the Rover
    public void SignalBrickDelivery()
    {
        if (currentState != LifterState.WaitingForRover)
        {
            Debug.LogWarning("Lifter received brick delivery signal but was not in WaitingForRover state. State: " + currentState);
            // Potentially queue or reject if busy
            return;
        }

        Debug.Log("Lifter: Brick delivery signalled by Rover.");

        brickVisualOnCrane.SetActive(true);
        brickVisualOnCrane.transform.SetParent(brickAnchor); // Parent to anchor for local animation

        // 2. Find a wall target
        currentWallTarget = FindNextAvailableWallTarget();

        if (currentWallTarget != null)
        {
            currentWallTarget.MarkAsReservedForPlacement();
            Debug.Log($"Lifter: Found wall target {currentWallTarget.name}. Preparing to move base.");

            // 3. Calculate position for Lifter base to approach the wall target
            // This needs to be tailored to your wall and Lifter's movement axis
            // Example: if lifter moves on XZ plane and Y is fixed (top of wall)
            // And target is ON the wall face. Lifter stops *in front* of it by approachOffsetDistance.
            Vector3 targetDirection = (currentWallTarget.transform.position - transform.position).normalized;
            targetDirection.y = 0; // Assuming movement is primarily on XZ plane for the base

            // Position the lifter base 'approachOffsetDistance' units away from the wall target,
            // along the line from the target to the lifter, or along a fixed axis.
            // For simplicity, let's assume the target's 'forward' points away from the wall.
            // So the lifter should be at target.position + target.forward * offset.
            Vector3 lifterBaseTargetPos = currentWallTarget.transform.position + currentWallTarget.transform.forward * approachOffsetDistance;
            lifterBaseTargetPos.y = transform.position.y; // Keep current height

            // If lifter moves only along one axis (e.g. X) and wall is along Z:
            // lifterBaseTargetPos = new Vector3(currentWallTarget.transform.position.x, transform.position.y, transform.position.z - approachOffsetDistance);

            SetNavDestination(lifterBaseTargetPos);
            currentState = LifterState.MovingBaseToWallTarget;
        }
        else
        {
            Debug.LogWarning("Lifter: Received brick, but no available wall targets found! Holding brick.");
            // Handle this case: maybe wait, or drop brick, or signal error
            // For now, it will just hold the brick and stay in WaitingForRover, effectively blocking.
            // To prevent blocking, it should go to a different state or clear the brick.
            // For this example, let's make it go back to waiting, which is not ideal if it's holding a brick.
            brickVisualOnCrane.SetActive(false); // Hide brick if no target
            brickVisualOnCrane.transform.SetParent(null);
            currentState = LifterState.WaitingForRover;
            Debug.Log("Lifter reverting to WaitingForRover as no target was found after brick delivery.");
        }
    }

    IEnumerator AnimateCraneToPlaceBrick(Target wallTarget)
    {
        if (isCraneAnimating) yield break;
        isCraneAnimating = true;

        // --- Phase 1: Lift and Orient brick locally on crane arm ---
        Debug.Log("Lifter: Crane Animation Phase 1 - Lifting and Orienting local brick.");
        Vector3 initialLocalPos = brickVisualOnCrane.transform.localPosition;
        Quaternion initialLocalRot = brickVisualOnCrane.transform.localRotation;

        Quaternion targetPhase1LocalRot = Quaternion.Euler(liftedBrickLocalRotationEuler);
        float journeyTime1 = 0f;
        float duration1 = Vector3.Distance(initialLocalPos, liftedBrickLocalPosition) / brickAnimationSpeed;
        if (duration1 <= 0.01f) duration1 = 0.1f; // Min duration

        while (journeyTime1 < duration1)
        {
            journeyTime1 += Time.deltaTime;
            float fraction = Mathf.Clamp01(journeyTime1 / duration1);
            brickVisualOnCrane.transform.localPosition = Vector3.Lerp(initialLocalPos, liftedBrickLocalPosition, fraction);
            brickVisualOnCrane.transform.localRotation = Quaternion.Slerp(initialLocalRot, targetPhase1LocalRot, fraction);
            yield return null;
        }
        brickVisualOnCrane.transform.localPosition = liftedBrickLocalPosition;
        brickVisualOnCrane.transform.localRotation = targetPhase1LocalRot;
        Debug.Log("Lifter: Crane Animation Phase 1 Complete.");
        yield return new WaitForSeconds(0.2f); // Small pause

        // --- Phase 2: Extend/Move brick to align with World Wall Target ---
        Debug.Log("Lifter: Crane Animation Phase 2 - Extending to Wall Target.");
        Vector3 currentPhase2LocalPos = brickVisualOnCrane.transform.localPosition;
        Quaternion currentPhase2LocalRot = brickVisualOnCrane.transform.localRotation;

        // Calculate the required *local* position and rotation for the brick (child of brickAnchor)
        // such that its *world* position and rotation match the wallTarget.
        Vector3 targetBrickWorldPos = wallTarget.transform.position;
        Quaternion targetBrickWorldRot = wallTarget.transform.rotation; // Assuming wall target defines final orientation

        Vector3 finalTargetLocalPos = brickAnchor.InverseTransformPoint(targetBrickWorldPos);
        Quaternion finalTargetLocalRot = Quaternion.Inverse(brickAnchor.rotation) * targetBrickWorldRot;

        float journeyTime2 = 0f;
        float duration2 = Vector3.Distance(currentPhase2LocalPos, finalTargetLocalPos) / brickAnimationSpeed;
         if (duration2 <= 0.01f) duration2 = 0.1f; // Min duration

        while (journeyTime2 < duration2)
        {
            journeyTime2 += Time.deltaTime;
            float fraction = Mathf.Clamp01(journeyTime2 / duration2);
            brickVisualOnCrane.transform.localPosition = Vector3.Lerp(currentPhase2LocalPos, finalTargetLocalPos, fraction);
            brickVisualOnCrane.transform.localRotation = Quaternion.Slerp(currentPhase2LocalRot, finalTargetLocalRot, fraction);
            yield return null;
        }
        // brickVisualOnCrane.transform.localPosition = finalTargetLocalPos; // Ensure it's exact
        // brickVisualOnCrane.transform.localRotation = finalTargetLocalRot;

        // // Brick is now visually in place. Finalize.
        // Debug.Log("Lifter: Crane Animation Phase 2 Complete. Brick placed at " + wallTarget.name);
        // brickVisualOnCrane.transform.SetParent(null); // Unparent from crane
        // // The brick visual is now permanent in the world at wallTarget's location.
        // // If you want the Target's own 'filledBrickVisual' to appear instead, then:
        // // brickVisualOnCrane.SetActive(false);

        // wallTarget.ConfirmPlacement();

        // isCraneAnimating = false;
        // currentState = LifterState.BrickPlacedCooldown;
        // Debug.Log("Lifter: Entering cooldown.");

        // yield return new WaitForSeconds(placementCooldown);

        // Debug.Log("Lifter: Cooldown finished. Returning to WaitingForRover.");
        // currentState = LifterState.WaitingForRover;
        // currentWallTarget = null; // Clear current target
    }


    void OnDrawGizmosSelected()
    {
        if (agent != null && agent.hasPath)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, agent.destination);
            Gizmos.DrawWireSphere(agent.destination, 0.3f);
        }
        if (roverHandoverPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(roverHandoverPoint.position, 0.25f);
            Gizmos.DrawIcon(roverHandoverPoint.position + Vector3.up * 0.5f, "UGUI_ArrowDropUp", true);
        }
        if (brickAnchor != null && brickVisualOnCrane != null && brickVisualOnCrane.activeSelf)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(brickVisualOnCrane.transform.position, 0.15f); // Show current brick pos
        }
        if (currentWallTarget != null)
        {
             Gizmos.color = Color.green;
             Gizmos.DrawLine(transform.position, currentWallTarget.transform.position);
        }
    }
}