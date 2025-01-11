using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class HideBehaviour : MonoBehaviour
{
    public Transform itemLocation;
    public Transform escapeLocation;
    public float detectionRadius = 6f;
    public float wanderRadius = 10f;
    private NavMeshAgent agent;
    private enum State { GoingToItem, Wandering, Fleeing, Escaping }

    private State currentState;

    private float wanderTimer;
    private float wanderInterval = 2f;

    private float fleeTimer;
    private float fleeInterval = 1f;

    public delegate void ItemStolenHandler();
    public static event ItemStolenHandler OnItemStolen;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        currentState = State.GoingToItem;
    }

    void Update()
    {
        switch (currentState)
        {
            case State.GoingToItem:
                GoToItem();
                break;
            case State.Wandering:
                Wander();
                break;
            case State.Fleeing:
                Flee();
                break;
            case State.Escaping:
                Escape();
                break;
        }
    }

    void GoToItem()
    {
        agent.destination = itemLocation.position;
        if (Vector3.Distance(transform.position, itemLocation.position) < 1f)
        {
            currentState = State.Escaping;
            agent.speed = 9f;
            detectionRadius = 20f;
            itemLocation.gameObject.SetActive(false);
            OnItemStolen?.Invoke();
        }
        else if (IsCopNearby())
        {
            currentState = State.Wandering;
        }
    }

    void Wander()
    {
        wanderTimer += Time.deltaTime;
        if (wanderTimer >= wanderInterval)
        {
            Vector3 randomDirection = Random.insideUnitSphere * wanderRadius;
            randomDirection += transform.position;
            NavMeshHit hit;
            NavMesh.SamplePosition(randomDirection, out hit, wanderRadius, 1);
            Vector3 finalPosition = hit.position;
            agent.destination = finalPosition;
            wanderTimer = 0f;
        }
        if (!IsCopNearby())
        {
            currentState = State.GoingToItem;
        }
    }

    void Flee()
    {
        fleeTimer += Time.deltaTime;
        if (fleeTimer >= fleeInterval)
        {
            Vector3 fleeDirection = transform.position - GetNearestCopPosition();
            Vector3 newFleePosition = transform.position + fleeDirection.normalized * 60;
            NavMeshHit hit;
            NavMesh.SamplePosition(newFleePosition, out hit, wanderRadius, 1);
            agent.destination = hit.position;
            fleeTimer = 0f;
        }
        if (!IsCopNearby())
        {
            currentState = State.Escaping;
        }
    }

    void Escape()
    {
        agent.destination = escapeLocation.position;

        if (IsCopNearby())
        {
            currentState = State.Fleeing;
        }

        if (Vector3.Distance(transform.position, escapeLocation.position) < 1f)
        {
            Destroy(gameObject);
        }
    }

    bool IsCopNearby()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius);
        foreach (var hitCollider in hitColliders)
        {
            if (hitCollider.CompareTag("Cop"))
            {
                return true;
            }
        }
        return false;
    }

    Vector3 GetNearestCopPosition()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius);
        foreach (var hitCollider in hitColliders)
        {
            if (hitCollider.CompareTag("Cop"))
            {
                return hitCollider.transform.position;
            }
        }
        return Vector3.zero;
    }
}
