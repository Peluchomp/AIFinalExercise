using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Rigidbody))]
public class AIDriver : MonoBehaviour
{
    public RacingCar racingCar;

    [Header("Car Settings")]
    public float maxSpeed = 2000f;
    public float accelerationForce = 1000f;
    public float steeringSensitivity = 5f;
    public float brakeForce = 1000f;

    [Header("Obstacle Avoidance")]
    public float detectionDistance = 20f;
    public float avoidanceDistance = 10f;
    public float detectionAngle = 25f;
    public float avoidanceAngle = 20f;
    public LayerMask obstacleLayer;
    public float brakingDistance = 6f;
    public float pathOffset = 10f;
    public float steeringSpeed = 3f;
    public float brakingSmoothness = 14f;

    private Rigidbody carRigidbody;

    // FSM States
    private enum State { RecklessDriving, Driving, Braking, CarInFront }
    private State currentState = State.RecklessDriving;
    private State previousState;

    private void Start()
    {
        carRigidbody = GetComponent<Rigidbody>();
        if (racingCar == null)
        {
            Debug.LogError("RacingCar reference is missing!");
        }

        Debug.Log($"Initial state: {currentState}");
    }

    private void FixedUpdate()
    {
        if (currentState != previousState)
        {
            Debug.Log($"Transitioned from {previousState} to {currentState}");
            HandleStateEntry(currentState);
            previousState = currentState;
        }

        switch (currentState)
        {
            case State.RecklessDriving:
                RecklessDrivingBehavior();
                CheckForStateTransition();
                break;

            case State.Driving:
                DrivingBehavior();
                CheckForStateTransition();
                break;

            case State.Braking:
                BrakingBehavior();
                CheckForStateTransition();
                break;

            case State.CarInFront:
                CarInFrontBehavior();
                CheckForStateTransition();
                break;
        }
    }

    private void HandleStateEntry(State state)
    {
        switch (state)
        {
            case State.RecklessDriving:
                maxSpeed = 2000f;
                accelerationForce = 1000f;
                brakeForce = 1000f;
                steeringSensitivity = 5f;
                steeringSpeed = 3f;
                brakingSmoothness = 14f;
                brakingDistance = 6f;
                if (racingCar != null) racingCar.desiredDistance = 14f;
                break;

            case State.Driving:
                maxSpeed = 1000;
                accelerationForce = 500;
                brakeForce = 1000f;
                steeringSensitivity = 5f;
                steeringSpeed = 3f;
                brakingSmoothness = 14f;
                brakingDistance = 6f;
                if (racingCar != null) racingCar.desiredDistance = 10f;
                break;

            case State.CarInFront:
                maxSpeed = 220;
                accelerationForce = 150;
                brakeForce = 800f;
                steeringSensitivity = 6f;
                steeringSpeed = 5f;
                brakingSmoothness = 10f;
                brakingDistance = 6f;
                if (racingCar != null) racingCar.desiredDistance = 17f;
                break;

            case State.Braking:
                brakingSmoothness = 7f;
                brakingDistance = 6f;
                brakeForce = 2000f;
                break;
        }
    }

    private void RecklessDrivingBehavior()
    {
        DriveTowardsWaypoint();
    }

    private void DrivingBehavior()
    {
        DriveTowardsWaypoint();
    }

    private void BrakingBehavior()
    {
        ApplySmoothBrakes(carRigidbody.velocity.magnitude);
    }

    private void CarInFrontBehavior()
    {
        if (racingCar == null || racingCar.currentWaypoint == null) return;

        Collider carInFront = null;
        Collider[] detectedObjects = Physics.OverlapSphere(transform.position, detectionDistance, obstacleLayer);

        foreach (Collider obj in detectedObjects)
        {
            if (obj.CompareTag("carsemaforon"))
            {
                carInFront = obj;
                break;
            }
        }

        if (carInFront == null) return;

        Vector3 toWaypoint = racingCar.currentWaypoint.position - transform.position;
        Vector3 toCarInFront = carInFront.transform.position - transform.position;

        Vector3 leftOffset = Vector3.Cross(Vector3.up, toCarInFront).normalized * pathOffset;
        Vector3 rightOffset = -leftOffset;

        Vector3 leftDetour = transform.position + leftOffset;
        Vector3 rightDetour = transform.position + rightOffset;

        Vector3 validLeftDetour = GetValidNavMeshPosition(leftDetour);
        Vector3 validRightDetour = GetValidNavMeshPosition(rightDetour);

        float leftPathLength = Vector3.Distance(validLeftDetour, racingCar.currentWaypoint.position);
        float rightPathLength = Vector3.Distance(validRightDetour, racingCar.currentWaypoint.position);

        Vector3 chosenDirection = (leftPathLength < rightPathLength) ? validLeftDetour : validRightDetour;

        Vector3 directionToDetour = (chosenDirection - transform.position).normalized;

        carRigidbody.MoveRotation(Quaternion.RotateTowards(carRigidbody.rotation, Quaternion.LookRotation(directionToDetour), steeringSpeed * Time.deltaTime));
        carRigidbody.AddForce(transform.forward * accelerationForce, ForceMode.Acceleration);
    }

    private void CheckForStateTransition()
    {
        Collider[] detectedObjects = Physics.OverlapSphere(transform.position, detectionDistance, obstacleLayer);

        bool hasDetectionObstacle = false;
        bool hasAvoidanceObstacle = false;
        bool hasCarInFront = false;

        foreach (Collider obj in detectedObjects)
        {
            Vector3 toObject = obj.transform.position - transform.position;
            float distanceToObject = toObject.magnitude;
            float angleToObject = Vector3.Angle(transform.forward, toObject);

            bool inDetectionCone = angleToObject <= detectionAngle && distanceToObject <= detectionDistance;
            bool inAvoidanceCone = angleToObject <= avoidanceAngle && distanceToObject <= avoidanceDistance;

            if (inAvoidanceCone)
            {
                if (obj.CompareTag("Agent") || obj.CompareTag("FlockMember") || obj.CompareTag("Robber") || obj.CompareTag("Cop"))
                {
                    currentState = State.Braking;
                    return;
                }

                if (obj.CompareTag("carsemaforon"))
                {
                    hasCarInFront = true;
                }
            }

            if (inDetectionCone)
            {
                if (obj.CompareTag("Agent") || obj.CompareTag("FlockMember") || obj.CompareTag("Robber") || obj.CompareTag("Cop"))
                {
                    hasDetectionObstacle = true;
                }
            }
        }

        if (hasCarInFront && !hasDetectionObstacle)
        {
            currentState = State.CarInFront;
            return;
        }

        if (hasDetectionObstacle)
        {
            currentState = State.Driving;
            return;
        }

        currentState = State.RecklessDriving;
    }

    private void DriveTowardsWaypoint()
    {
        if (racingCar == null || racingCar.currentWaypoint == null) return;

        Vector3 targetPosition = GetValidNavMeshPosition(racingCar.currentWaypoint.position);
        Vector3 directionToWaypoint = (targetPosition - transform.position).normalized;

        Debug.DrawLine(transform.position, targetPosition, Color.green);

        float angle = Vector3.SignedAngle(transform.forward, directionToWaypoint, Vector3.up);
        float steer = Mathf.Clamp(angle / 45f, -1f, 1f);

        carRigidbody.MoveRotation(carRigidbody.rotation * Quaternion.Euler(0, steer * steeringSensitivity, 0));

        float currentSpeed = carRigidbody.velocity.magnitude;
        if (currentSpeed < maxSpeed)
        {
            carRigidbody.AddForce(transform.forward * accelerationForce, ForceMode.Acceleration);
        }
    }

    private void ApplySmoothBrakes(float currentSpeed)
    {
        if (currentSpeed > 0)
        {
            float targetSpeed = currentSpeed * (1 - brakingSmoothness);
            float brakeForceApplied = Mathf.Min(brakeForce, currentSpeed - targetSpeed);
            carRigidbody.AddForce(-carRigidbody.velocity.normalized * brakeForceApplied, ForceMode.Acceleration);
        }
    }

    private Vector3 GetValidNavMeshPosition(Vector3 position)
    {
        if (NavMesh.SamplePosition(position, out NavMeshHit hit, 5f, NavMesh.AllAreas))
        {
            return hit.position;
        }
        return position;
    }

    private void OnDrawGizmos()
    {
        // Detection Distance and Angle
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, detectionDistance);
        Vector3 detectionLeftEdge = Quaternion.Euler(0, -detectionAngle, 0) * transform.forward * detectionDistance;
        Vector3 detectionRightEdge = Quaternion.Euler(0, detectionAngle, 0) * transform.forward * detectionDistance;
        Gizmos.DrawLine(transform.position, transform.position + detectionLeftEdge);
        Gizmos.DrawLine(transform.position, transform.position + detectionRightEdge);

        // Avoidance Distance and Angle
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, avoidanceDistance);
        Vector3 avoidanceLeftEdge = Quaternion.Euler(0, -avoidanceAngle, 0) * transform.forward * avoidanceDistance;
        Vector3 avoidanceRightEdge = Quaternion.Euler(0, avoidanceAngle, 0) * transform.forward * avoidanceDistance;
        Gizmos.DrawLine(transform.position, transform.position + avoidanceLeftEdge);
        Gizmos.DrawLine(transform.position, transform.position + avoidanceRightEdge);

        // Braking Distance
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, brakingDistance);

        // Path Offset Visualization
        Gizmos.color = Color.green;
        Vector3 toWaypoint = racingCar != null && racingCar.currentWaypoint != null
            ? (racingCar.currentWaypoint.position - transform.position).normalized
            : transform.forward;
        Vector3 pathOffsetDirection = Vector3.Cross(toWaypoint, Vector3.up).normalized * pathOffset;
        Gizmos.DrawLine(transform.position, transform.position + pathOffsetDirection);

        // Current Waypoint
        if (racingCar != null && racingCar.currentWaypoint != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, racingCar.currentWaypoint.position);
            Gizmos.DrawWireSphere(racingCar.currentWaypoint.position, 1f);
        }
    }
}
