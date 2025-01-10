using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class WanderScript : MonoBehaviour
{
    public float wanderRadius = 50f;
    public float agentAvoidanceRadius = 5f;

    private NavMeshAgent agent;

    void OnEnable()
    {
        agent = GetComponent<NavMeshAgent>();
        agent.avoidancePriority = Random.Range(0, 100);
        SetNewDestination();
    }

    void Update()
    {
        if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance && (!agent.hasPath || agent.velocity.sqrMagnitude == 0f))
        {
            SetNewDestination();
        }
    }

    void SetNewDestination()
    {
        Vector3 newPos;
        int maxAttempts = 10;
        int attempts = 0;

        do
        {
            newPos = RandomNavSphere(transform.position, wanderRadius, -1);
            attempts++;
        }
        while (IsPositionNearAgents(newPos) && attempts < maxAttempts);

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

    bool IsPositionNearAgents(Vector3 position)
    {
        Collider[] hitColliders = Physics.OverlapSphere(position, agentAvoidanceRadius);
        foreach (var collider in hitColliders)
        {
            if (collider.CompareTag("Agent") && collider.gameObject != gameObject)
            {
                return true;
            }
        }
        return false;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, agentAvoidanceRadius);
    }
}
