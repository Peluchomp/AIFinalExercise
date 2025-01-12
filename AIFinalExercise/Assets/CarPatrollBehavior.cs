using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarPatrollBehavior : MonoBehaviour
{
    public Transform[] points;
    private int destPoint = 0;
    private UnityEngine.AI.NavMeshAgent agent;
    private bool isStopped = false;

    void Start()
    {
        agent = GetComponent<UnityEngine.AI.NavMeshAgent>();
        agent.autoBraking = false;
        GotoNextPoint();
    }

    void GotoNextPoint()
    {
        if (points.Length == 0)
            return;

        agent.destination = points[destPoint].position;
        destPoint = (destPoint + 1) % points.Length;
    }

    void Update()
    {
        if (!agent.pathPending && agent.remainingDistance < 0.5f && !isStopped)
        {
            GotoNextPoint();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Agent") || other.CompareTag("Red"))
        {
            isStopped = true;
            agent.isStopped = true;
            //Debug.Log("lalalal");
        }
        if (other.CompareTag("Green"))
        {
            isStopped = false;
            agent.isStopped = false;
         
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Green") && !other.CompareTag("Agent"))
        {
            isStopped = false;
            agent.isStopped = false;
           
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Agent") || other.CompareTag("Red"))
        {
            isStopped = false;
            agent.isStopped = false;
        
        }
    }
}
