using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class AdvancedFlockBehavior : MonoBehaviour
{
    public GameObject leader; // The leader object
    public float followDistance = 5f; // Distance flock members should maintain from the leader
    public float separationDistance = 2f; // Minimum distance between flock members
    public float cohesionStrength = 1f; // Cohesion force strength
    public float separationStrength = 1.5f; // Separation force strength
    public float alignmentStrength = 1f; // Alignment force strength
    public float wanderRadius = 10f; // Radius for random wandering behavior
    public float moveSpeed = 5f; // Movement speed of flock members
    public float avoidanceRadius = 5f; // Radius for avoiding other agents

    private NavMeshAgent leaderAgent;
    private List<GameObject> flockMembers;

    void Start()
    {
        if (leader == null)
        {
            Debug.LogError("Leader not assigned!");
            return;
        }

        leaderAgent = leader.GetComponent<NavMeshAgent>();
        if (leaderAgent == null)
        {
            Debug.LogError("Leader must have a NavMeshAgent component!");
            return;
        }

        flockMembers = new List<GameObject>(GameObject.FindGameObjectsWithTag("FlockMember"));
        flockMembers.Remove(leader); // Ensure the leader isn't included in the flock
    }

    void Update()
    {
        foreach (var member in flockMembers)
        {
            Vector3 cohesionVector = GetCohesionVector(member);
            Vector3 separationVector = GetSeparationVector(member);
            Vector3 alignmentVector = GetAlignmentVector(member);
            Vector3 followLeaderVector = GetFollowLeaderPath(member);

            // Combine forces
            Vector3 flockingForce = cohesionVector * cohesionStrength
                                  + separationVector * separationStrength
                                  + alignmentVector * alignmentStrength
                                  + followLeaderVector;

            if (!MoveMember(member, flockingForce))
            {
                Wander(member);
            }
        }
    }

    private Vector3 GetCohesionVector(GameObject member)
    {
        Vector3 centerOfMass = Vector3.zero;
        foreach (var otherMember in flockMembers)
        {
            if (otherMember != member)
                centerOfMass += otherMember.transform.position;
        }
        centerOfMass /= (flockMembers.Count - 1);
        return (centerOfMass - member.transform.position).normalized;
    }

    private Vector3 GetSeparationVector(GameObject member)
    {
        Vector3 separationForce = Vector3.zero;
        foreach (var otherMember in flockMembers)
        {
            if (otherMember != member)
            {
                float distance = Vector3.Distance(member.transform.position, otherMember.transform.position);
                if (distance < separationDistance)
                {
                    separationForce += member.transform.position - otherMember.transform.position;
                }
            }
        }
        return separationForce.normalized;
    }

    private Vector3 GetAlignmentVector(GameObject member)
    {
        Vector3 averageDirection = Vector3.zero;
        foreach (var otherMember in flockMembers)
        {
            if (otherMember != member)
                averageDirection += otherMember.transform.forward;
        }
        averageDirection /= (flockMembers.Count - 1);
        return averageDirection.normalized;
    }

    private Vector3 GetFollowLeaderPath(GameObject member)
    {
        NavMeshPath path = new NavMeshPath();
        Vector3 followPosition = leader.transform.position;

        // Calculate the optimal path to the leader
        if (NavMesh.CalculatePath(member.transform.position, followPosition, NavMesh.AllAreas, path))
        {
            if (path.corners.Length > 1)
            {
                // Move towards the first corner of the path
                return (path.corners[1] - member.transform.position).normalized * followDistance;
            }
        }

        // If no path can be calculated, fallback to a direct vector
        return (followPosition - member.transform.position).normalized * followDistance;
    }

    private bool MoveMember(GameObject member, Vector3 force)
    {
        NavMeshAgent agent = member.GetComponent<NavMeshAgent>();
        if (agent != null)
        {
            Vector3 targetPosition = member.transform.position + force;

            if (NavMesh.SamplePosition(targetPosition, out NavMeshHit hit, wanderRadius, NavMesh.AllAreas))
            {
                agent.SetDestination(hit.position);
                return true;
            }
        }
        else
        {
            member.transform.position += force * moveSpeed * Time.deltaTime;
        }
        return false;
    }

    private void Wander(GameObject member)
    {
        NavMeshAgent agent = member.GetComponent<NavMeshAgent>();
        if (agent != null)
        {
            Vector3 wanderTarget = RandomNavSphere(member.transform.position, wanderRadius, -1);
            agent.SetDestination(wanderTarget);
        }
    }

    private Vector3 RandomNavSphere(Vector3 origin, float dist, int layermask)
    {
        Vector3 randDirection = Random.insideUnitSphere * dist;
        randDirection += origin;

        NavMesh.SamplePosition(randDirection, out NavMeshHit navHit, dist, layermask);
        return navHit.position;
    }

    void OnDrawGizmosSelected()
    {
        if (flockMembers != null)
        {
            Gizmos.color = Color.green;
            foreach (var member in flockMembers)
            {
                Gizmos.DrawWireSphere(member.transform.position, avoidanceRadius);
            }
        }
    }
}
