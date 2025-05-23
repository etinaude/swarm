using UnityEngine;

public class Target : MonoBehaviour
{
    public enum TargetState { NotPlaced, ReservedForPlacement, Filled }
    public TargetState currentState = TargetState.NotPlaced;
    public Renderer objectRenderer;


    void Start()
    {
        if (objectRenderer != null)
            objectRenderer.enabled = false;
        if (objectRenderer == null) Debug.LogWarning("NO RENDERER");
    }

    public void MarkAsReservedForPlacement()
    {
        if (currentState == TargetState.NotPlaced)
        {
            currentState = TargetState.ReservedForPlacement;
            Debug.Log($"{gameObject.name} is now reserved for placement.");
        }
        else
        {
            Debug.LogWarning($"{gameObject.name} tried to reserve, but state is {currentState}");
        }
    }

    public void ConfirmPlacement()
    {
        currentState = TargetState.Filled;
        if (objectRenderer != null) objectRenderer.enabled = true;
        Debug.Log($"{gameObject.name} has been successfully filled.");
    }

    public bool IsAvailable()
    {
        return currentState == TargetState.NotPlaced;
    }
}