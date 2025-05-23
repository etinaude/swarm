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
        if (brickObject == null) Debug.LogError("Lifter: Brick Visual On Crane is not assigned!");
        if (brickAnchor == null) Debug.LogError("Lifter: Brick Anchor transform is not assigned!");
        if (roverHandoverPoint == null) Debug.LogError("Lifter: Rover Handover Point (child of Lifter) is not assigned!");
        else if (roverHandoverPoint.parent != transform) Debug.LogWarning("Lifter: roverHandoverPoint should be a direct child of the Lifter GameObject for correct positioning.");

        Debug.Log("Lifter: Started");



        brickObject.SetActive(false);
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

                ChangeState(LifterState.MovingBaseToWallTarget);
                // Stop any existing movement coroutines on this object to prevent conflicts
                StopAllCoroutines();

                // Start the new movement coroutine
                StartCoroutine(MoveAlongZAxisCoroutine());
                yield break; // Exit coroutine once target is found and moving
            }
            else
            {
                Debug.Log("Lifter: No available wall targets found. Will retry in 3 seconds.");
                yield return new WaitForSeconds(3f); // Wait before retrying
            }
        }
    }

    private IEnumerator MoveAlongZAxisCoroutine()
    {
        // Store the starting position of this object
        Vector3 startPosition = transform.position;

        // Get the Z-coordinate of the target wall
        float targetZ = currentWallTarget.position.z;

        // Calculate the final Z position based on the direction of movement
        // If the target is "further" in Z, we stop before it.
        // If the target is "closer" in Z, we stop after it.
        float finalZ;
        if (targetZ > startPosition.z) // Moving towards positive Z
        {
            finalZ = targetZ - stopDistanceZ;
        }
        else // Moving towards negative Z
        {
            finalZ = targetZ + stopDistanceZ;
        }

        // The target position for Vector3.Lerp, keeping X and Y constant
        // We only want to change the Z component of the current object's position.
        Vector3 endPositionForLerp = new Vector3(startPosition.x, startPosition.y, finalZ);

        // Calculate the absolute distance to travel along the Z axis
        float distanceToTravel = Mathf.Abs(finalZ - startPosition.z);

        // Calculate the duration of the movement based on distance and speed
        // This ensures consistent speed regardless of the distance.
        float duration = distanceToTravel / movementSpeed;

        float journeyTime = 0f;

        // Only proceed if there's actual distance to cover
        if (duration > 0)
        {
            // Loop while the journey time is less than the calculated duration
            while (journeyTime < duration)
            {
                journeyTime += Time.deltaTime; // Increment time by the time passed since last frame
                float fraction = Mathf.Clamp01(journeyTime / duration); // Calculate progress (0 to 1)

                // Interpolate the Z position, keeping X and Y from the starting position
                // We use startPosition.x and startPosition.y to ensure movement is strictly on Z
                Vector3 currentLerpedPosition = Vector3.Lerp(startPosition, endPositionForLerp, fraction);
                transform.position = new Vector3(startPosition.x, startPosition.y, currentLerpedPosition.z);

                yield return null; // Wait for the next frame before continuing the loop
            }
        }

        // Ensure the object is exactly at the final Z position to avoid floating point inaccuracies
        transform.position = new Vector3(startPosition.x, startPosition.y, finalZ);
    }

    void OnNavBaseArrival()
    {
        Debug.Log("Lifter base arrived at approach point for: " + currentWallTarget.name);
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