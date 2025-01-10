using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class WanderScript : MonoBehaviour
{
    public float wanderRadius = 50f; // Larger radius for wider exploration

    private NavMeshAgent agent;

    void OnEnable()
    {
        agent = GetComponent<NavMeshAgent>();
        SetNewDestination();
    }

    void Update()
    {
        // Check if the agent has reached the destination
        if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance && (!agent.hasPath || agent.velocity.sqrMagnitude == 0f))
        {
            SetNewDestination();
        }
    }

    void SetNewDestination()
    {
        Vector3 newPos = RandomNavSphere(transform.position, wanderRadius, -1);
        agent.SetDestination(newPos);
    }

    public static Vector3 RandomNavSphere(Vector3 origin, float dist, int layermask)
    {
        Vector3 randDirection = Random.insideUnitSphere * dist;

        randDirection += origin;

        NavMeshHit navHit;
        NavMesh.SamplePosition(randDirection, out navHit, dist, layermask);

        return navHit.position;
    }
}
