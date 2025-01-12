using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Rigidbody))]
public class AIDriver : MonoBehaviour
{
    public RacingCar racingCar;
    public float maxSpeed = 15f;
    public float accelerationForce = 10f;
    public float steeringSensitivity = 2f;
    public float brakeForce = 30f;
    public float waypointThreshold = 3f;
    public float dodgeRadius = 5f;
    public float dodgeForce = 20f;

    private Rigidbody carRigidbody;

    private void Start()
    {
        carRigidbody = GetComponent<Rigidbody>();
        if (racingCar == null)
        {
            Debug.LogError("RacingCar reference is missing!");
        }
    }

    private void FixedUpdate()
    {
        if (racingCar.currentWaypoint == null)
        {
            Debug.LogWarning("No current waypoint set!");
            return;
        }

        DebugGroundContact();
        AvoidNavMeshAgents();
        DriveTowardsWaypoint();
        CheckWaypointReached();
    }

    private void DebugGroundContact()
    {
        Ray ray = new Ray(transform.position, Vector3.down);
        if (!Physics.Raycast(ray, out RaycastHit hit, 1.5f))
        {
            Debug.LogWarning("Car is not grounded!");
        }
    }

    private void AvoidNavMeshAgents()
    {
        Collider[] nearbyAgents = Physics.OverlapSphere(transform.position, dodgeRadius, LayerMask.GetMask("NavMeshAgent"));

        foreach (Collider agent in nearbyAgents)
        {
            Vector3 toAgent = agent.transform.position - transform.position;
            Vector3 dodgeDirection = -toAgent.normalized;

            carRigidbody.AddForce(dodgeDirection * dodgeForce, ForceMode.Acceleration);
            Debug.DrawRay(transform.position, dodgeDirection * 5f, Color.red);
        }
    }

    private void DriveTowardsWaypoint()
    {
        Vector3 targetPosition = racingCar.currentWaypoint.position;
        Vector3 directionToWaypoint = (targetPosition - transform.position).normalized;

        // Debug lines
        Debug.DrawLine(transform.position, targetPosition, Color.green);
        Debug.DrawRay(transform.position, transform.forward * 5f, Color.blue);

        // Steering
        Vector3 forward = transform.forward;
        float angle = Vector3.SignedAngle(forward, directionToWaypoint, Vector3.up);
        float steer = Mathf.Clamp(angle / 45f, -1f, 1f);
        carRigidbody.MoveRotation(carRigidbody.rotation * Quaternion.Euler(0, steer * steeringSensitivity, 0));

        // Acceleration
        float speed = carRigidbody.velocity.magnitude;
        if (Mathf.Abs(angle) < 30f && speed < maxSpeed) // Accelerate when aligned
        {
            carRigidbody.AddForce(transform.forward * accelerationForce, ForceMode.Acceleration);
            Debug.Log("Applying forward force.");
        }
        else if (speed > maxSpeed || Mathf.Abs(angle) > 60f) // Brake when overshooting
        {
            carRigidbody.AddForce(-carRigidbody.velocity.normalized * brakeForce, ForceMode.Acceleration);
            Debug.Log("Applying brake force.");
        }
    }

    private void CheckWaypointReached()
    {
        Vector3 targetPosition = racingCar.currentWaypoint.position;
        float distanceToWaypoint = Vector3.Distance(transform.position, targetPosition);

        if (distanceToWaypoint < waypointThreshold)
        {
            racingCar.calculateDistanceofWaypoints();
        }
    }
}
