using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class FlockBehavior : MonoBehaviour
{
    public GameObject leader; // Assign the leader object here
    public float followDistance = 5f; // Desired distance from the leader
    public float separationDistance = 2f; // Desired distance between flock members
    public float cohesionStrength = 1f; // Strength of cohesion behavior
    public float separationStrength = 1.5f; // Strength of separation behavior
    public float alignmentStrength = 1f; // Strength of alignment behavior
    public float moveSpeed = 5f; // Speed of flock members

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

            Vector3 followLeaderVector = (leader.transform.position - member.transform.position).normalized * followDistance;

            Vector3 flockingForce = cohesionVector * cohesionStrength
                                  + separationVector * separationStrength
                                  + alignmentVector * alignmentStrength
                                  + followLeaderVector;

            MoveMember(member, flockingForce);
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

    private void MoveMember(GameObject member, Vector3 force)
    {
        NavMeshAgent agent = member.GetComponent<NavMeshAgent>();
        if (agent != null)
        {
            Vector3 targetPosition = member.transform.position + force;
            agent.SetDestination(targetPosition);
        }
        else
        {
            member.transform.position += force * moveSpeed * Time.deltaTime;
        }
    }
}
