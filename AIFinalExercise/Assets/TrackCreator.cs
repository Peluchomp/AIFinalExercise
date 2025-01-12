using System.Collections.Generic;
using UnityEngine;

public class trackWaypoints : MonoBehaviour
{
    public Color linecolor = Color.red; // Bright color for visibility
    [Range(0, 1)] public float SphereRadius = 0.5f; // Larger sphere radius for testing

    public List<Transform> nodes = new List<Transform>();

    public void OnDrawGizmosSelected()
    {
        if (!Application.isEditor) return; // Ensure Gizmos only draw in the editor

        Gizmos.color = linecolor;

        // Gather all children transforms
        Transform[] path = GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for (int i = 1; i < path.Length; i++)
        {
            nodes.Add(path[i]);
        }

        // Draw lines and spheres
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector3 currentWaypoint = nodes[i].position;
            Vector3 previousWaypoint = i == 0
                ? nodes[nodes.Count - 1].position
                : nodes[i - 1].position;

            Gizmos.DrawLine(previousWaypoint, currentWaypoint); // Draw line
            Gizmos.DrawSphere(currentWaypoint, SphereRadius);  // Draw sphere

            Debug.Log($"Drawing Gizmo at Waypoint {i}: {currentWaypoint}");
        }
    }
}
