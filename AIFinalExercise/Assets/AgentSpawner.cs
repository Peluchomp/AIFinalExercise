using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class AgentSpawner : MonoBehaviour
{
    public GameObject[] agentPrefabs; // Array of prefabs to choose from
    public int numberOfAgents = 200; // Number of agents to spawn
    public float spawnRadius = 100f; // Radius around the spawner to spawn agents
    public NavMeshData navMeshData; // Specific NavMeshData to use

    void Start()
    {
        if (navMeshData != null)
        {
            SpawnAgents();
        }
        else
        {
            Debug.LogError("No NavMeshData assigned! Assign a target NavMeshData in the inspector.");
        }
    }

    void SpawnAgents()
    {
        for (int i = 0; i < numberOfAgents; i++)
        {
            Vector3 spawnPosition = GetRandomPointOnNavMesh(transform.position, spawnRadius);
            if (spawnPosition != Vector3.zero)
            {
                GameObject randomPrefab = agentPrefabs[Random.Range(0, agentPrefabs.Length)];
                Instantiate(randomPrefab, spawnPosition, Quaternion.identity);
            }
        }
    }

    Vector3 GetRandomPointOnNavMesh(Vector3 origin, float radius)
    {
        Vector3 randomDirection = Random.insideUnitSphere * radius;
        randomDirection += origin;

        NavMeshHit hit;
        // Ensure we sample using the specified NavMeshData
        if (NavMesh.SamplePosition(randomDirection, out hit, radius, NavMesh.AllAreas))
        {
            // Check if the sampled position is within the bounds of the specified NavMeshData
            if (navMeshData.sourceBounds.Contains(hit.position))
            {
                return hit.position;
            }
        }

        return Vector3.zero; // Return zero if no valid position is found
    }
}
