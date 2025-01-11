using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SeekBehaviour : MonoBehaviour
{
    public Transform itemLocation;
    public float detectionRadius = 20;
    public float wanderRadius = 30f;
    private UnityEngine.AI.NavMeshAgent agent;
    private enum State { Wandering, Chasing }
    private State currentState;
    private Transform robber;
    private float wanderTimer;
    private float wanderInterval = 4f;

    void Start()
    {
        agent = GetComponent<UnityEngine.AI.NavMeshAgent>();
        currentState = State.Wandering;
        HideBehaviour.OnItemStolen += StartChasing;
    }

    void OnDestroy()
    {
        HideBehaviour.OnItemStolen -= StartChasing;
    }

    void Update()
    {
        switch (currentState)
        {
            case State.Wandering:
                Wander();
                break;
            case State.Chasing:
                Chase();
                break;
        }
    }

    void Wander()
    {
        wanderTimer += Time.deltaTime;
        if (wanderTimer >= wanderInterval)
        {
            Vector3 randomDirection = Random.insideUnitSphere * wanderRadius;
            randomDirection += itemLocation.position;
            UnityEngine.AI.NavMeshHit hit;
            UnityEngine.AI.NavMesh.SamplePosition(randomDirection, out hit, wanderRadius, 1);
            Vector3 finalPosition = hit.position;
            agent.destination = finalPosition;
            wanderTimer = 0f;
        }
    }

    void Chase()
    {
        if (robber != null)
        {
            agent.destination = robber.position;
            if (Vector3.Distance(transform.position, robber.position) > detectionRadius)
            {
                currentState = State.Wandering;
            }
        }
        else
        {
            currentState = State.Wandering;
        }
    }

    void StartChasing()
    {
        currentState = State.Chasing;
        robber = GameObject.FindGameObjectWithTag("Robber").transform;
        agent.speed = 7f;
        wanderRadius = 60f;
        detectionRadius = 15f;
    }
}
