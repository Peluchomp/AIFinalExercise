using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class AIDriver : MonoBehaviour
{
    public RacingCar racingCar;

    [Header("Car Settings")]
    public float maxSpeed = 15f;
    public float accelerationForce = 10f;
    public float steeringSensitivity = 2f;
    public float brakeForce = 30f;

    [Header("Obstacle Avoidance")]
    public float avoidanceDistance = 10f; // Maximum distance for obstacle detection
    public float avoidanceAngle = 8f; // Initial angle of the detection cone
    public LayerMask obstacleLayer; // Layer for detecting obstacles
    public float brakingDistance = 5f; // Distance at which to brake if an obstacle is too close
    public float pathOffset = 1f; // Maximum distance from the path where the car should avoid obstacles
    public float clearPathDistance = 10f; // Minimum distance for a clear path
    public float steeringSpeed = 2f; // Speed of steering rotation to make it smoother
    public float slowDownFactor = 0.5f; // Factor to apply slight slow down while avoiding obstacles
    public float brakingSmoothness = 0.1f; // Smoothness factor for braking

    private Rigidbody carRigidbody;
    private bool isAvoiding = false; // Flag to check if the car is avoiding an obstacle
    private float dynamicAvoidanceAngle;

    private void Start()
    {
        carRigidbody = GetComponent<Rigidbody>();
        if (racingCar == null)
        {
            Debug.LogError("RacingCar reference is missing!");
        }
        dynamicAvoidanceAngle = avoidanceAngle; // Initialize dynamic avoidance angle
    }

    private void FixedUpdate()
    {
        if (racingCar.currentWaypoint == null)
        {
            Debug.LogWarning("No current waypoint set!");
            return;
        }

        DebugGroundContact();
        AvoidObstacles();

        // If we're not avoiding an obstacle, drive towards the waypoint
        if (!isAvoiding)
        {
            DriveTowardsWaypoint();
        }

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

    private void DriveTowardsWaypoint()
    {
        Vector3 targetPosition = racingCar.currentWaypoint.position;
        Vector3 directionToWaypoint = (targetPosition - transform.position).normalized;

        Debug.DrawLine(transform.position, targetPosition, Color.green);
        Debug.DrawRay(transform.position, transform.forward * 5f, Color.blue);

        Vector3 forward = transform.forward;
        float angle = Vector3.SignedAngle(forward, directionToWaypoint, Vector3.up);
        float steer = Mathf.Clamp(angle / 45f, -1f, 1f);
        carRigidbody.MoveRotation(carRigidbody.rotation * Quaternion.Euler(0, steer * steeringSensitivity, 0));

        float speed = carRigidbody.velocity.magnitude;
        if (Mathf.Abs(angle) < 30f && speed < maxSpeed)
        {
            carRigidbody.AddForce(transform.forward * accelerationForce, ForceMode.Acceleration);
        }
        else if (speed > maxSpeed || Mathf.Abs(angle) > 60f)
        {
            // Apply smoother braking
            ApplySmoothBrakes(speed);
        }
    }

    private void ApplySmoothBrakes(float currentSpeed)
    {
        // Apply a gradual reduction of speed by a percentage
        if (currentSpeed > 0)
        {
            float targetSpeed = currentSpeed * (1 - brakingSmoothness);
            float brakeForceApplied = Mathf.Min(brakeForce, currentSpeed - targetSpeed);
            carRigidbody.AddForce(-carRigidbody.velocity.normalized * brakeForceApplied, ForceMode.Acceleration);
        }
    }

    private void AvoidObstacles()
    {
        // Get all colliders in the detection cone
        Collider[] nearbyObjects = Physics.OverlapSphere(transform.position, avoidanceDistance, obstacleLayer);

        isAvoiding = false; // Reset the flag each frame
        dynamicAvoidanceAngle = avoidanceAngle; // Reset to initial avoidance angle each frame

        foreach (Collider obj in nearbyObjects)
        {
            // If the object has the relevant tags (including "Agent" or "FlockMember")
            if (obj.CompareTag("Agent") || obj.CompareTag("carsemaforon") || obj.CompareTag("FlockMember"))
            {
                Vector3 toObstacle = obj.transform.position - transform.position;
                float distanceToObstacle = toObstacle.magnitude;

                // Check if the obstacle is within the cone
                if (IsInAvoidanceCone(toObstacle, distanceToObstacle))
                {
                    // If the obstacle is near the path, avoid it
                    isAvoiding = true;

                    // Widen the avoidance cone to understand the situation better
                    dynamicAvoidanceAngle = Mathf.Lerp(avoidanceAngle, 60f, distanceToObstacle / avoidanceDistance);

                    // Apply smooth braking and wait for the path to be clear
                    ApplySmoothBrakes(carRigidbody.velocity.magnitude);
                    break; // Stop further steering or acceleration until the path is clear
                }

                // Draw line to detected obstacle
                Debug.DrawLine(transform.position, obj.transform.position, Color.red);
            }
        }

        // Once the obstacle is avoided, return to normal avoidance cone size
        if (!isAvoiding)
        {
            dynamicAvoidanceAngle = avoidanceAngle;
        }
    }

    private bool IsInAvoidanceCone(Vector3 toObstacle, float distanceToObstacle)
    {
        // Check if the obstacle is within the cone's angle and distance
        float angleToObstacle = Vector3.Angle(transform.forward, toObstacle);

        if (angleToObstacle <= dynamicAvoidanceAngle && distanceToObstacle <= avoidanceDistance)
        {
            return true;
        }

        return false;
    }

    private void RotateToAvoid(Vector3 toObstacle, float distanceToObstacle)
    {
        // Calculate a direction to steer the car away from the obstacle
        Vector3 avoidanceDirection = (Vector3.Cross(transform.up, toObstacle)).normalized;

        // Gradually steer the car using Quaternion.RotateTowards
        Quaternion targetRotation = Quaternion.LookRotation(avoidanceDirection);
        carRigidbody.MoveRotation(Quaternion.RotateTowards(carRigidbody.rotation, targetRotation, steeringSpeed * Time.deltaTime));

        // Apply slight braking based on the distance to the obstacle (only if necessary)
        if (distanceToObstacle <= avoidanceDistance * 0.7f) // If too close, slow down a bit
        {
            float speed = carRigidbody.velocity.magnitude;
            if (speed > 0)
            {
                carRigidbody.AddForce(-carRigidbody.velocity.normalized * speed * slowDownFactor, ForceMode.Acceleration);
            }
        }
    }

    private bool CheckClearPath()
    {
        // Raycast ahead to check if the path is clear
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, clearPathDistance, obstacleLayer))
        {
            // If we hit something within clearPathDistance, return false (path is not clear)
            return false;
        }
        return true; // Path is clear
    }

    private void CheckWaypointReached()
    {
        Vector3 targetPosition = racingCar.currentWaypoint.position;
        float distanceToWaypoint = Vector3.Distance(transform.position, targetPosition);

        if (distanceToWaypoint < racingCar.desiredDistance)
        {
            racingCar.calculateDistanceofWaypoints();
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, avoidanceDistance); // Show the avoidance distance
        Gizmos.color = Color.red;
        // Draw the cone
        Vector3 leftEdge = Quaternion.Euler(0, -dynamicAvoidanceAngle, 0) * transform.forward * avoidanceDistance;
        Vector3 rightEdge = Quaternion.Euler(0, dynamicAvoidanceAngle, 0) * transform.forward * avoidanceDistance;
        Gizmos.DrawLine(transform.position, transform.position + leftEdge);
        Gizmos.DrawLine(transform.position, transform.position + rightEdge);
    }
}
