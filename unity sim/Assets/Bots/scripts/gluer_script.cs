using UnityEngine;
using System.Collections;

#if UNITY_EDITOR
    using UnityEditor;
    #endif

public class Gluer : MonoBehaviour
{
    public enum GluerState { Idle, Gluing, DoneGluing }
    public GluerState currentState = GluerState.Idle;

    [Header("Visuals")]
    public GameObject brickObject; // Gluer's visual of the brick being worked on
    public GameObject glueObject; // Visual for glue application

    public float gluingTime = 5f;

    void Start()
    {
        if(brickObject != null) brickObject.SetActive(false);
        if(glueObject != null) glueObject.SetActive(false);
        Debug.Log("Gluer Online. State: " + currentState);
    }

    public void StartGluingProcess()
    {
        if (currentState == GluerState.Idle)
        {
            currentState = GluerState.Gluing;
            Debug.Log("Gluer: Starting gluing process.");
            if(brickObject != null) brickObject.SetActive(true); // Show brick
            StartCoroutine(GluingCoroutine());
        }
    }

    IEnumerator GluingCoroutine()
    {
        if(glueObject != null) glueObject.SetActive(false); // Show glue effect
        yield return new WaitForSeconds(gluingTime);
        if(glueObject != null) glueObject.SetActive(true);
        // Brick still visible, now with conceptual glue
        currentState = GluerState.DoneGluing;
        Debug.Log("Gluer: Gluing process complete. Ready for Rover pickup.");
    }

    public void RoverCollectsBrick()
    {
        if (currentState == GluerState.DoneGluing)
        {
            Debug.Log("Gluer: Rover collected glued brick.");
            if(brickObject != null) brickObject.SetActive(false); // Hide brick
            if(glueObject != null) glueObject.SetActive(false); // Hide glue effect
            currentState = GluerState.Idle; // Ready for next brick
        }
    }

    void OnDrawGizmos()
    {
        Handles.Label(transform.position + Vector3.up * 0.5f, $"Gluer State: {currentState}");
    }
}