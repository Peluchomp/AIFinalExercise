using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Wander : MonoBehaviour
{
    private NavMeshAgent agent;
    [SerializeField] private float wanderRadius = 15f;
    [SerializeField] private float wanderTimer = 8f;
    private float timer;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        timer = wanderTimer;

        // Hardcoded starting position
        Vector3 startPosition = new Vector3(-8f, 0.2f, 2f);
        transform.position = startPosition;
        agent.Warp(startPosition); // Align agent to the NavMesh at the hardcoded position
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= wanderTimer)
        {
            Vector3 newDestination = RandomNavMeshLocation(wanderRadius);

            if (agent.isOnNavMesh)
            {
                agent.SetDestination(newDestination);
            }

            timer = 0;
        }

        if (agent.remainingDistance < 0.5f && !agent.pathPending)
        {
            Vector3 unstuckPosition = RandomNavMeshLocation(wanderRadius);
            agent.SetDestination(unstuckPosition);
        }
    }

    private Vector3 RandomNavMeshLocation(float radius)
    {
        Vector3 randomDirection = Random.insideUnitSphere * radius;
        randomDirection.y = 0; // Keep movement on the horizontal plane
        randomDirection += transform.position;

        NavMeshHit hit;
        if (NavMesh.SamplePosition(randomDirection, out hit, radius, NavMesh.AllAreas))
        {
            return hit.position;
        }

        return transform.position;
    }
}
