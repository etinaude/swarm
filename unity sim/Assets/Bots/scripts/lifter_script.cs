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
        Idle
    }
    public LifterState currentState = LifterState.Idle;

    [Header("References")]
    public GameObject groundBrickObject;
    public GameObject brickObject;
    public Transform brickAnchor;
    public Transform craneRotation;


    [Header("Rover Interaction")]
    public Transform roverHandoverPoint;

    [Header("Crane Animation Settings")]
    public float brickAnimationSpeed = 1.0f;
    public float rotationAnimationSpeed = 100f;
    public float placementCooldown = 1.8f;

    [Header("Wall Placement Settings")]
    public Target currentWallTarget;
    private bool isCraneAnimating = false;
    private Coroutine currentLifterCoroutine;
    private float stopDistanceZ = 2f;
    private float movementSpeed = 0.8f;

    void Start()
    {
        if (brickObject == null) Debug.LogError("Lifter: Brick Visual On Crane is not assigned!");
        if (brickAnchor == null) Debug.LogError("Lifter: Brick Anchor transform is not assigned!");
        if (roverHandoverPoint == null) Debug.LogError("Lifter: Rover Handover Point (child of Lifter) is not assigned!");
        else if (roverHandoverPoint.parent != transform) Debug.LogWarning("Lifter: roverHandoverPoint should be a direct child of the Lifter GameObject for correct positioning.");

        Debug.Log("Lifter: Started");

        brickObject.SetActive(false);
        groundBrickObject.SetActive(false);
        currentState = LifterState.Idle;
        ChangeState(LifterState.FindingWallTarget);
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
                break;
            case LifterState.WaitingForRoverAtWall:
                break;
            case LifterState.PositioningCraneAndPlacing:
                break;
        }
    }

    IEnumerator FindingWallTargetRoutine()
    {
        while (true)
        {
            Target[] listOfTargets = FindObjectsOfType<Target>() as Target[];

            Debug.Log("Targets: " + listOfTargets.Length);

            currentWallTarget = FindObjectsOfType<Target>()
            .Where(t => t.IsAvailable())
            .OrderBy(t => Vector3.Distance(transform.position, t.transform.position))
            .LastOrDefault();

            if (currentWallTarget != null)
            {
                currentWallTarget.MarkAsReservedForPlacement();
                Debug.Log($"Lifter: Found wall target {currentWallTarget.name}. Preparing to move base.");

                ChangeState(LifterState.MovingBaseToWallTarget);
                StopAllCoroutines();
                StartCoroutine(MoveAlongZAxisCoroutine());
                yield break;
            }
            else
            {
                Debug.Log("Lifter: No available wall targets found. Will retry in 3 seconds.");
                yield return new WaitForSeconds(3f);
            }
        }
    }

    private IEnumerator MoveAlongZAxisCoroutine()
    {
        Vector3 startPosition = transform.position;

        float targetZ = currentWallTarget.transform.position.z;

        float finalZ;
        if (targetZ > startPosition.z)
            finalZ = targetZ - stopDistanceZ;
        else
            finalZ = targetZ + stopDistanceZ;

        Vector3 endPositionForLerp = new Vector3(startPosition.x, startPosition.y, finalZ);

        float distanceToTravel = Mathf.Abs(finalZ - startPosition.z);
        float duration = distanceToTravel / movementSpeed;
        float journeyTime = 0f;

        if (duration > 0)
        {
            while (journeyTime < duration)
            {
                journeyTime += Time.deltaTime;
                float fraction = Mathf.Clamp01(journeyTime / duration);

                Vector3 currentLerpedPosition = Vector3.Lerp(startPosition, endPositionForLerp, fraction);
                transform.position = new Vector3(startPosition.x, startPosition.y, currentLerpedPosition.z);

                yield return null;
            }
        }

        // transform.position = new Vector3(startPosition.x, startPosition.y, finalZ);
        Debug.Log("Lifter base arrived at approach point for: " + currentWallTarget.name);
        ChangeState(LifterState.WaitingForRoverAtWall);
    }

    public void SignalBrickDeliveryFromRover()
    {
        if (currentState != LifterState.WaitingForRoverAtWall)
        {
            Debug.LogWarning($"Lifter: Received brick delivery signal but was not in WaitingForRoverAtWall state. Current state: {currentState}");
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
        groundBrickObject.SetActive(true);
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
        groundBrickObject.SetActive(false);


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
        currentWallTarget = null;
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

        isCraneAnimating = false;
        ChangeState(LifterState.FindingWallTarget);
    }
}