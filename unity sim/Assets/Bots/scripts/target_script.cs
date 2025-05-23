using UnityEngine;

public class Target : MonoBehaviour
{
    public enum TargetState { NotPlaced, ReservedForPlacement, Filled }
    public TargetState currentState = TargetState.NotPlaced;

    public GameObject placementIndicator; // Optional: Visual to show this is the next placement spot
    public GameObject filledBrickVisual;  // Optional: A visual to activate when this spot is filled

    void Start()
    {
        if (placementIndicator != null) placementIndicator.SetActive(false);
        if (filledBrickVisual != null) filledBrickVisual.SetActive(false);
        UpdateVisuals();
    }

    // Called by Lifter when it chooses this as its destination for placement
    public void MarkAsReservedForPlacement()
    {
        if (currentState == TargetState.NotPlaced)
        {
            currentState = TargetState.ReservedForPlacement;
            if (placementIndicator != null) placementIndicator.SetActive(true);
            Debug.Log($"{gameObject.name} is now reserved for placement.");
            UpdateVisuals();
        }
        else
        {
            Debug.LogWarning($"{gameObject.name} tried to reserve, but state is {currentState}");
        }
    }

    // Called by Lifter when it has successfully placed its brick at this Target's location
    public void ConfirmPlacement()
    {
        currentState = TargetState.Filled;
        if (placementIndicator != null) placementIndicator.SetActive(false);
        if (filledBrickVisual != null)
        {
            filledBrickVisual.SetActive(true);
            // Optional: if the lifter passes its brick, position it here.
            // For now, just activate this target's own filled visual.
        }
        Debug.Log($"{gameObject.name} has been successfully filled.");
        UpdateVisuals();
    }

    public bool IsAvailable()
    {
        return currentState == TargetState.NotPlaced;
    }

    void UpdateVisuals() // Basic visual feedback
    {
        Renderer rend = GetComponent<Renderer>(); // Main renderer of the target slot itself
        if (rend != null)
        {
            switch (currentState)
            {
                case TargetState.NotPlaced:
                    rend.material.color = Color.gray;
                    break;
                case TargetState.ReservedForPlacement:
                    rend.material.color = Color.yellow;
                    break;
                case TargetState.Filled:
                    rend.material.color = Color.green;
                    break;
            }
        }
    }

    void OnValidate()
    {
        // Update visuals in editor if state changes, helps with setup
        if (placementIndicator != null && currentState != TargetState.ReservedForPlacement && Application.isEditor && !Application.isPlaying) placementIndicator.SetActive(false);
        if (placementIndicator != null && currentState == TargetState.ReservedForPlacement && Application.isEditor && !Application.isPlaying) placementIndicator.SetActive(true);
        if (filledBrickVisual != null && currentState != TargetState.Filled && Application.isEditor && !Application.isPlaying) filledBrickVisual.SetActive(false);
        if (filledBrickVisual != null && currentState == TargetState.Filled && Application.isEditor && !Application.isPlaying) filledBrickVisual.SetActive(true);
    }
}