using UnityEngine;
using UnityEngine.AI;
using System.Linq;

public class Rover : MonoBehaviour
{
    public enum RoverState { Idle, GettingBrickFromPile, MovingToGluer, WaitingForGlue, MovingToLifter, DeliveringToLifter }
    public RoverState currentState = RoverState.Idle;

    public NavMeshAgent agent;
    public Gluer currentGluer;
    public Lifter currentLifter;

    [Header("Object References")]
    public GameObject brickPile;
    public GameObject brickVisual; // Rover's brick visual
    public GameObject glueVisual;  // Rover's glue visual

    [Header("Navigation")]
    public Vector3 currentNavTargetPos;

    void Start()
    {
        if (agent == null) agent = GetComponent<NavMeshAgent>();
        if (brickVisual != null) brickVisual.SetActive(false);
        if (glueVisual != null) glueVisual.SetActive(false);
        Debug.Log("Rover Online. State: " + currentState);
    }

    void SetNavDestination(Vector3 pos)
    {
        currentNavTargetPos = pos;
        agent.SetDestination(currentNavTargetPos);
        agent.isStopped = false;
    }

    void Update()
    {
        UpdateStateMachine();

        if (!agent.pathPending && agent.hasPath && !agent.isStopped)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                OnNavArrival();
            }
        }
    }

    void UpdateStateMachine()
    {
        switch (currentState)
        {
            case RoverState.Idle:
                Debug.Log("Rover: Idle. Going to get brick.");
                currentState = RoverState.GettingBrickFromPile;
                if (brickPile != null)
                {
                    SetNavDestination(brickPile.transform.position);
                }
                else Debug.LogError("Rover: Brick Pile not assigned!");
                break;

            case RoverState.WaitingForGlue:
                if (currentGluer != null && currentGluer.currentState == Gluer.GluerState.DoneGluing)
                {
                    Debug.Log("Rover: Gluer is done. Collecting glued brick.");
                    currentGluer.RoverCollectsBrick(); // Tell gluer it's been collected
                    if(brickVisual != null) brickVisual.SetActive(true);
                    if(glueVisual != null) glueVisual.SetActive(true); // Show glue on brick

                    currentLifter = FindWaitingLifter();
                    if (currentLifter != null)
                    {
                        Debug.Log($"Rover: Found waiting lifter: {currentLifter.name}. Moving to its handover point.");
                        SetNavDestination(currentLifter.roverHandoverPoint.position);
                        currentState = RoverState.MovingToLifter;
                    }
                    else
                    {
                        Debug.LogWarning("Rover: No waiting lifter found. Will retry or wait.");
                    }
                }
                break;
        }
    }

    void OnNavArrival()
    {
        Debug.Log($"Rover: Arrived at NavTarget. Current state: {currentState}");
        agent.isStopped = true;
        // agent.ResetPath(); // Can be useful

        switch (currentState)
        {
            case RoverState.GettingBrickFromPile:
                Debug.Log("Rover: Arrived at brick pile. Picked up a brick.");
                if(brickVisual != null) brickVisual.SetActive(true); // Show brick

                currentGluer = FindIdleGluer();
                if (currentGluer != null)
                {
                    Debug.Log($"Rover: Found idle gluer: {currentGluer.name}. Moving to gluer.");
                    SetNavDestination(currentGluer.transform.position);
                    currentState = RoverState.MovingToGluer;
                }
                else
                {
                    Debug.LogWarning("Rover: No idle gluer found. Waiting.");
                    // Stay in this state or go to a waiting state, will retry next frame
                }
                break;

            case RoverState.MovingToGluer:
                Debug.Log("Rover: Arrived at gluer.");
                if (currentGluer != null)
                {
                    currentGluer.StartGluingProcess();
                    if(brickVisual != null) brickVisual.SetActive(false);
                    currentState = RoverState.WaitingForGlue;
                }
                break;

            case RoverState.MovingToLifter:
                Debug.Log("Rover: Arrived at Lifter's handover point.");
                if (currentLifter != null)
                {
                    currentLifter.SignalBrickDelivery(); // Signal lifter
                    if(brickVisual != null) brickVisual.SetActive(false); // Deactivate its visuals
                    if(glueVisual != null) glueVisual.SetActive(false);
                    currentState = RoverState.Idle; // Rover is done with this cycle
                    currentLifter = null; // Release reference
                    Debug.Log("Rover: Brick delivered to Lifter. Returning to Idle.");
                }
                break;
        }
    }

    Lifter FindWaitingLifter()
    {
        return FindObjectsOfType<Lifter>()
            .Where(l => l.currentState == Lifter.LifterState.WaitingForRover)
            .OrderBy(l => Vector3.Distance(transform.position, l.transform.position))
            .FirstOrDefault();
    }

    Gluer FindIdleGluer()
    {
        return FindObjectsOfType<Gluer>()
            .Where(g => g.currentState == Gluer.GluerState.Idle)
            .OrderBy(g => Vector3.Distance(transform.position, g.transform.position))
            .FirstOrDefault();
    }

     void OnDrawGizmosSelected()
    {
        if (agent != null && agent.hasPath)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, agent.destination);
        }
         if (currentGluer != null && (currentState == RoverState.MovingToGluer || currentState == RoverState.WaitingForGlue))
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(transform.position, currentGluer.transform.position);
        }
        if (currentLifter != null && currentState == RoverState.MovingToLifter)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, currentLifter.roverHandoverPoint.position);
        }
    }
}