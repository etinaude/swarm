using UnityEngine;
using System.Linq;
using System.Collections;
using UnityEngine.AI;

public class Lifter : MonoBehaviour
{
    public enum LifterState
    {
        FindingWallTarget,
        MovingBaseToWallTarget,
        WaitingForRoverAtWall,
        PositioningCraneAndPlacing, // Combines receiving and placing animation
        BrickPlacedCooldown,
        Idle
    }
    public LifterState currentState = LifterState.Idle;

    [Header("References")]
    public NavMeshAgent agent;
    public GameObject brickVisualOnCrane;
    public Transform brickAnchor; // Parent for the brickVisual, crane's "hook"
    public Transform craneRotation; // Parent for the brickVisual, crane's "hook"

    public Vector3 defaultRotation = Vector3(0, 230, 0);
    public Vector3 outerBrickRotation = Vector3(0, 160, 0);
    public Vector3 innerBrickRotation = Vector3(0, 120, 0);
    public Vector3 brickOnGround = Vector3(0, -0.0005, -0.00208);
    public Vector3 brickInAir = Vector3(-0.00146, -0.00384, -0.00208);



    [Header("Rover Interaction")]
    // IMPORTANT: Assign an empty child GameObject of this Lifter.
    // This point moves with the lifter.
    public Transform roverHandoverPoint;

    [Header("Crane Animation Settings")]
    public float brickAnimationSpeed = 1.0f;
    public float placementCooldown = 2.0f;

    [Header("Wall Placement Settings")]
    public float approachOffsetToWallTarget = 0.5f; // How far from the wall target center the Lifter base stops. Adjust based on crane reach.

    private Target currentWallTarget;
    private bool isCraneAnimating = false; // To prevent re-triggering animation
    private Coroutine currentLifterCoroutine; // To manage ongoing operations

    void Start()
    {
        // --- Critical Null Checks ---
        if (agent == null) agent = GetComponent<NavMeshAgent>();
        if (agent == null) Debug.LogError("Lifter: NavMeshAgent component not found or assigned!");
        if (brickVisualOnCrane == null) Debug.LogError("Lifter: Brick Visual On Crane is not assigned!");
        if (brickAnchor == null) Debug.LogError("Lifter: Brick Anchor transform is not assigned!");
        if (roverHandoverPoint == null) Debug.LogError("Lifter: Rover Handover Point (child of Lifter) is not assigned!");
        else if (roverHandoverPoint.parent != transform) Debug.LogWarning("Lifter: roverHandoverPoint should be a direct child of the Lifter GameObject for correct positioning.");

        Debug.Log("Lifter: Started");



        brickVisualOnCrane.SetActive(false);
        agent.updateRotation = false;
        // For straight movement along a wall, you might want to disable NavMeshAgent's rotation updates
        // agent.updateRotation = false; // And handle Lifter's body rotation manually or keep it fixed.
        currentState = LifterState.Idle;
        ChangeState(LifterState.FindingWallTarget);
    }

    void Update()
    {
        // State-specific continuous logic (if any) can go here,
        // but most transitions are event-driven or coroutine-driven.
        if (currentState == LifterState.MovingBaseToWallTarget)
        {
            CheckNavArrival();
        }
    }

    void ChangeState(LifterState newState)
    {
        Debug.Log("Lifter: State change");
        Debug.Log($"Lifter current state: {currentState}");
        Debug.Log($"Lifter new state: {newState}");


        if (currentState == newState) return;

        currentState = newState;
        Debug.Log($"Lifter State changed to: {currentState}");

        // --- State Entry Logic ---
        switch (currentState)
        {
            case LifterState.FindingWallTarget:
                if (currentLifterCoroutine != null) StopCoroutine(currentLifterCoroutine);
                Debug.Log("lifter: find wall target");

                currentLifterCoroutine = StartCoroutine(FindingWallTargetRoutine());
                break;
            case LifterState.MovingBaseToWallTarget:
                // Movement is initiated by FindingWallTargetRoutine
                break;
            case LifterState.WaitingForRoverAtWall:
                // Lifter is now positioned and ready. Rover will find it.
                Debug.Log($"Lifter is at {transform.position}, waiting for Rover at {roverHandoverPoint.position} for target {currentWallTarget?.name}");
                break;
            case LifterState.PositioningCraneAndPlacing:
                // Animation is started by SignalBrickDelivery
                break;
            case LifterState.BrickPlacedCooldown:
                // Cooldown is handled at the end of the placement coroutine
                break;
        }
    }

    IEnumerator FindingWallTargetRoutine()
    {
        Debug.Log("Lifter: WALLLLL");

        while (true) // Keep trying to find a target
        {
            currentWallTarget = FindNextAvailableWallTarget();

            if (currentWallTarget != null)
            {
                currentWallTarget.MarkAsReservedForPlacement();
                Debug.Log($"Lifter: Found wall target {currentWallTarget.name}. Preparing to move base.");

                // Calculate position for Lifter base to approach the wall target
                // This needs to be tailored: if wall is along X, lifter moves on Z, or vice-versa.
                // Assuming wallTarget.transform.forward points "out" from the wall face.
                // Lifter stops 'approachOffsetToWallTarget' in front of it.
                Vector3 lifterBaseTargetPos = currentWallTarget.transform.position;
                lifterBaseTargetPos.y = transform.position.y; // Maintain Lifter's current height (assuming it's on a track/wall top)

                SetNavDestination(lifterBaseTargetPos);
                ChangeState(LifterState.MovingBaseToWallTarget);
                yield break; // Exit coroutine once target is found and moving
            }
            else
            {
                Debug.Log("Lifter: No available wall targets found. Will retry in 3 seconds.");
                yield return new WaitForSeconds(3f); // Wait before retrying
            }
        }
    }


    void SetNavDestination(Vector3 pos)
    {
        agent.SetDestination(pos);
        agent.isStopped = false;
    }

    void CheckNavArrival()
    {
        if (!agent.pathPending && agent.hasPath && !agent.isStopped)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                OnNavBaseArrival();
            }
        }
    }

    void OnNavBaseArrival()
    {
        Debug.Log("Lifter base arrived at approach point for: " + currentWallTarget.name);
        agent.isStopped = true;
        ChangeState(LifterState.WaitingForRoverAtWall);
        // SignalBrickDeliveryFromRover();
    }

    Target FindNextAvailableWallTarget()
    {
        return FindObjectsOfType<Target>()
            .Where(t => t.IsAvailable())
            // Adjust OrderBy if you have a specific sequence for wall building
            .OrderBy(t => Vector3.Distance(transform.position, t.transform.position))
            .LastOrDefault();
    }

    // Called by the Rover when it arrives at roverHandoverPoint
    public void SignalBrickDeliveryFromRover()
    {
        if (currentState != LifterState.WaitingForRoverAtWall)
        {
            Debug.LogWarning($"Lifter: Received brick delivery signal but was not in WaitingForRoverAtWall state. Current state: {currentState}");
            // Optionally, Rover could check Lifter's state before even moving to it.
            return;
        }

        if (isCraneAnimating)
        {
            Debug.LogWarning("Lifter: Crane is already animating. Ignoring new brick signal.");
            return;
        }

        Debug.Log("Lifter: Brick delivery signalled by Rover.");

        // 1. Activate crane's brick visual at the (now correctly positioned) handover point.
        // The roverHandoverPoint is a child of the Lifter, so its world position is correct.
        brickVisualOnCrane.transform.position = roverHandoverPoint.position;
        brickVisualOnCrane.SetActive(true);
        brickVisualOnCrane.transform.SetParent(brickAnchor); // Parent to anchor for local animation.
        ChangeState(LifterState.PositioningCraneAndPlacing);
        if (currentLifterCoroutine != null) StopCoroutine(currentLifterCoroutine);
        currentLifterCoroutine = StartCoroutine(AnimateCraneFromHandoverToPlaceAtWallTarget(currentWallTarget));
    }

    IEnumerator AnimateCraneFromHandoverToPlaceAtWallTarget(Target wallTarget)
    {
        isCraneAnimating = true;
        brickAnchor.transform.localPosition = new Vector3(0.486f, 0f, 0);

        // --- Phase 1: Secure brick and small initial lift/orientation (relative to anchor) ---
        Debug.Log("Lifter: Crane Animation Phase 1 - Securing and lifting brick from handover.");
        Vector3 initialLocalPosOnAnchor = brickAnchor.transform.localPosition; // Position right after parenting at handover
        Vector3 UpPos = new Vector3(0.8f, 2f, 0); // lift up height

        float journeyTime1 = 0f;
        // Base duration on the offset, or use a fixed small time
        float duration1 = (UpPos - initialLocalPosOnAnchor).magnitude / brickAnimationSpeed;

        if (duration1 > 0)
        {
            while (journeyTime1 < duration1)
            {
                journeyTime1 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime1 / duration1);
                brickAnchor.transform.localPosition = Vector3.Lerp(initialLocalPosOnAnchor, UpPos, fraction);
                yield return null;
            }
        }
        brickAnchor.transform.localPosition = UpPos;
        Debug.Log("Lifter: Crane Animation Phase 1 Complete (Secured/Lifted from Handover).");
        if (duration1 > 0) yield return new WaitForSeconds(0.3f); // Small pause if Phase 1 happened


        // --- Phase 2: Extend/Move brick to align with World Wall Target ---
        Debug.Log("Lifter: Crane Animation Phase 2 - Rotate to Wall Target: " + wallTarget.name);
        Vector3 currentPhase2StartLocalPos = brickAnchor.transform.localPosition; // Current pos after phase 1

        Vector3 finalTargetLocalPosForBrick = new Vector3(1.6f, 1.2f, 2f);

        float journeyTime2 = 0f;
        float duration2 = Vector3.Distance(currentPhase2StartLocalPos, finalTargetLocalPosForBrick) / brickAnimationSpeed;
        if (duration2 <= 0.01f) duration2 = 0.1f;

        while (journeyTime2 < duration2)
        {
            journeyTime2 += Time.deltaTime;
            float fraction = Mathf.Clamp01(journeyTime2 / duration2);
            brickAnchor.transform.localPosition = Vector3.Lerp(currentPhase2StartLocalPos, finalTargetLocalPosForBrick, fraction);
            yield return null;
        }
        brickAnchor.transform.localPosition = finalTargetLocalPosForBrick;

        Debug.Log("Lifter: Crane Animation Phase 2 Complete. Brick placed at " + wallTarget.name);


        wallTarget.ConfirmPlacement();
        currentWallTarget = null; // Clear current wall target
        brickVisualOnCrane.SetActive(false);
        brickAnchor.transform.position = roverHandoverPoint.position;


        // --- Phase 3: Rotate Crane back  ---
        float journeyTime3 = 0f;
        float duration3 = Vector3.Distance(finalTargetLocalPosForBrick, UpPos) / brickAnimationSpeed;
        if (duration3 <= 0.01f) duration3 = 0.1f;

        while (journeyTime3 < duration3)
        {
            journeyTime3 += Time.deltaTime;
            float fraction = Mathf.Clamp01(journeyTime3 / duration3);
            brickAnchor.transform.localPosition = Vector3.Lerp(finalTargetLocalPosForBrick, UpPos, fraction);
            yield return null;
        }
        brickAnchor.transform.localPosition = UpPos;

        Debug.Log("Lifter: Crane Animation Phase 3 Complete. ");


        isCraneAnimating = false;
        ChangeState(LifterState.FindingWallTarget); // Go back to find the next spot
    }

    void OnDrawGizmosSelected()
    {
        if (agent != null && agent.hasPath && agent.destination != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, agent.destination);
            Gizmos.DrawWireSphere(agent.destination, 0.3f);
        }
        if (roverHandoverPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(roverHandoverPoint.position, 0.25f); // Shows world position
            Gizmos.DrawIcon(roverHandoverPoint.position + Vector3.up * 0.5f, "d_MoveTool On", true); // Simpler icon
        }
        // if (brickAnchor != null && brickVisualOnCrane != null && brickVisualOnCrane.activeSelf)
        // {
        //     Gizmos.color = Color.red;
        //     Gizmos.DrawSphere(brickVisualOnCrane.transform.position, 0.15f);
        // }
        if (currentWallTarget != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, currentWallTarget.transform.position);
            Gizmos.DrawWireCube(currentWallTarget.transform.position, Vector3.one * 0.4f); // Mark target
        }

    }
}