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
        PositioningCraneAndPlacing,
        BrickPlacedCooldown,
        Idle
    }
    public LifterState currentState = LifterState.Idle;

    [Header("References")]
    public NavMeshAgent agent;
    public GameObject brickObject;
    public Transform brickAnchor;
    public Transform craneRotation;





    [Header("Rover Interaction")]
    public Transform roverHandoverPoint;

    [Header("Crane Animation Settings")]
    public float brickAnimationSpeed = 1.0f;
    public float rotationAnimationSpeed = 100f;
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
        if (brickObject == null) Debug.LogError("Lifter: Brick Visual On Crane is not assigned!");
        if (brickAnchor == null) Debug.LogError("Lifter: Brick Anchor transform is not assigned!");
        if (roverHandoverPoint == null) Debug.LogError("Lifter: Rover Handover Point (child of Lifter) is not assigned!");
        else if (roverHandoverPoint.parent != transform) Debug.LogWarning("Lifter: roverHandoverPoint should be a direct child of the Lifter GameObject for correct positioning.");

        Debug.Log("Lifter: Started");



        brickObject.SetActive(false);
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
        SignalBrickDeliveryFromRover();
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

        ChangeState(LifterState.PositioningCraneAndPlacing);
        if (currentLifterCoroutine != null) StopCoroutine(currentLifterCoroutine);
        currentLifterCoroutine = StartCoroutine(AnimateCraneFromHandoverToPlaceAtWallTarget(currentWallTarget));
    }

    IEnumerator AnimateCraneFromHandoverToPlaceAtWallTarget(Target wallTarget)
    {

        Quaternion defaultRotation = Quaternion.Euler(0, 230, 0);
        Quaternion outerBrickRotation = Quaternion.Euler(0, 160, 0);
        Quaternion innerBrickRotation = Quaternion.Euler(0, 120, 0);
        Vector3 brickOnGround = new Vector3(3.109f, 3.465f, -0.629f);
        Vector3 brickInAir = new Vector3(2.95f, 1.4f, -0.796f);
        Vector3 brickInPlace = new Vector3(2.95f, 2f, -0.796f);


        isCraneAnimating = true;
        brickAnchor.transform.localPosition = brickInAir;
        craneRotation.transform.localRotation = defaultRotation;


        Debug.Log("Lifter: Crane Animation Phase 0 - get brick");

        float journeyTime0 = 0f;
        float duration0 = (brickInAir - brickOnGround).magnitude / brickAnimationSpeed;

        if (duration0 > 0)
        {
            while (journeyTime0 < duration0)
            {
                journeyTime0 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime0 / duration0);
                brickAnchor.transform.localPosition = Vector3.Lerp(brickInAir, brickOnGround, fraction);
                yield return null;
            }
        }
        brickAnchor.transform.localPosition = brickOnGround;
        brickObject.SetActive(true);

        Debug.Log("Lifter: Crane Animation Phase 0 Complete");
        if (duration0 > 0) yield return new WaitForSeconds(0.3f);

        Debug.Log("Lifter: Crane Animation Phase 1 - Securing and lifting brick from handover.");

        float journeyTime1 = 0f;
        float duration1 = (brickInAir - brickOnGround).magnitude / brickAnimationSpeed;

        if (duration1 > 0)
        {
            while (journeyTime1 < duration1)
            {
                journeyTime1 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime1 / duration1);
                brickAnchor.transform.localPosition = Vector3.Lerp(brickOnGround, brickInAir, fraction);
                yield return null;
            }
        }
        brickAnchor.transform.localPosition = brickInAir;
        Debug.Log("Lifter: Crane Animation Phase 1 Complete");
        if (duration1 > 0) yield return new WaitForSeconds(0.3f);



        // --- Phase 2: Rotate arm to target ---
        Debug.Log("Lifter: Crane Animation Phase 2 - Rotate to Wall Target: " + wallTarget.name);
        float journeyTime2 = 0f;
        float duration2 = Quaternion.Angle(defaultRotation, outerBrickRotation) / rotationAnimationSpeed;

        if (duration2 > 0)
        {
            while (journeyTime2 < duration2)
            {
                journeyTime2 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime2 / duration2);
                craneRotation.transform.localRotation = Quaternion.Slerp(defaultRotation, outerBrickRotation, fraction);
                yield return null;
            }
        }
        craneRotation.transform.localRotation = outerBrickRotation;
        Debug.Log("Lifter: Crane Animation Phase 2 Complete.");
        if (duration2 > 0) yield return new WaitForSeconds(0.3f);


        // --- Phase 3: place the brick ---
        Debug.Log("Lifter: Crane Animation Phase 3");

        float journeyTime3 = 0f;
        float duration3 = (brickInAir - brickInPlace).magnitude / brickAnimationSpeed;

        if (duration3 > 0)
        {
            while (journeyTime3 < duration3)
            {
                journeyTime3 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime3 / duration3);
                brickAnchor.transform.localPosition = Vector3.Lerp(brickInAir, brickInPlace, fraction);
                yield return null;
            }
        }
        brickAnchor.transform.localPosition = brickInPlace;
        Debug.Log("Lifter: Crane Animation Phase 3 Complete Placed.");
        if (duration3 > 0) yield return new WaitForSeconds(0.3f);

        wallTarget.ConfirmPlacement();
        currentWallTarget = null; // Clear current wall target
        brickObject.SetActive(false);


        // --- Phase 4: lift to neutral the brick ---
        Debug.Log("Lifter: Crane Animation Phase 4");

        float journeyTime4 = 0f;
        float duration4 = (brickInAir - brickInPlace).magnitude / brickAnimationSpeed;

        if (duration4 > 0)
        {
            while (journeyTime4 < duration4)
            {
                journeyTime4 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime4 / duration4);
                brickAnchor.transform.localPosition = Vector4.Lerp(brickInPlace, brickInAir, fraction);
                yield return null;
            }
        }
        brickAnchor.transform.localPosition = brickInAir;
        Debug.Log("Lifter: Crane Animation Phase 4 Complete Placed.");
        if (duration4 > 0) yield return new WaitForSeconds(0.3f);



        // --- Phase 5: Rotate arm to normal ---
        Debug.Log("Lifter: Crane Animation Phase 5 - Rotate to Wall Target: " + wallTarget.name);
        float journeyTime5 = 0f;
        float duration5 = Quaternion.Angle(defaultRotation, outerBrickRotation) / rotationAnimationSpeed;

        if (duration5 > 0)
        {
            while (journeyTime5 < duration5)
            {
                journeyTime5 += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime5 / duration5);
                craneRotation.transform.localRotation = Quaternion.Slerp(outerBrickRotation, defaultRotation, fraction);
                yield return null;
            }
        }
        craneRotation.transform.localRotation = defaultRotation;
        Debug.Log("Lifter: Crane Animation Phase 5 Complete.");
        if (duration5 > 0) yield return new WaitForSeconds(0.3f);


        // isCraneAnimating = false;
        // ChangeState(LifterState.FindingWallTarget); // Go back to find the next spot
    }
}