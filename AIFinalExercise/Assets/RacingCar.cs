using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RacingCar : MonoBehaviour
{
    public trackWaypoints waypoints;
    public float desiredDistance = 10f; // The desired minimum distance to the next waypoint
    public Transform currentWaypoint;
    public List<Transform> nodes = new List<Transform>();

    private int currentWaypointIndex = 0;

    public void Awake()
    {
        waypoints = GameObject.FindGameObjectWithTag("Path")?.GetComponent<trackWaypoints>();
        if (waypoints == null)
        {
            Debug.LogError("Path object or trackWaypoints component not found!");
            return;
        }
        nodes = waypoints.nodes;

        Debug.Log($"Nodes loaded: {nodes.Count}");
        if (nodes.Count == 0)
        {
            Debug.LogError("Waypoints list is empty! Check trackWaypoints script.");
        }
    }

    public void Start()
    {
        if (nodes.Count > 0)
        {
            currentWaypoint = nodes[currentWaypointIndex];
        }
    }

    public void Update()
    {
        if (nodes.Count == 0 || currentWaypoint == null) return;

        calculateDistanceofWaypoints();
    }

    public void calculateDistanceofWaypoints()
    {
        Vector3 carPosition = transform.position;
        float distanceToWaypoint = Vector3.Distance(carPosition, currentWaypoint.position);

        // Skip to the next waypoint if the car is too close
        while (distanceToWaypoint < desiredDistance)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % nodes.Count;
            currentWaypoint = nodes[currentWaypointIndex];
            distanceToWaypoint = Vector3.Distance(carPosition, currentWaypoint.position);
        }

        Debug.Log($"Current Waypoint: {currentWaypoint.name}, Distance: {distanceToWaypoint}");
    }

    private void OnDrawGizmos()
    {
        if (currentWaypoint != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(currentWaypoint.position, 3f);
        }
    }
}
