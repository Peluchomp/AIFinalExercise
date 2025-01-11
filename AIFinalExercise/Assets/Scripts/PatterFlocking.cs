using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PatterFlocking : MonoBehaviour
{
    public Transform leader;
    public float leaderSpeed = 5f;
    public float agentSpeed = 5f;
    public float formationSpacing = 2f;
    public float smoothTime = 0.2f;
    public float circleRadius = 10f;

    public List<Transform> agents;

    private List<Vector3> formationOffsets;
    private Dictionary<Transform, Vector3> velocities = new Dictionary<Transform, Vector3>();
    private float angle = 0f;

    void Start()
    {
        if (leader == null || agents.Count == 0)
        {
            Debug.LogError("Leader and agents must be assigned!");
            return;
        }

        formationOffsets = CalculateVFormationOffsets();

        foreach (var agent in agents)
        {
            velocities[agent] = Vector3.zero;
        }
    }

    void Update()
    {
        MoveLeaderInCircle();

        for (int i = 0; i < agents.Count; i++)
        {
            MoveAgentToFormation(agents[i], formationOffsets[i]);
        }
    }

    void MoveLeaderInCircle()
    {
        angle += leaderSpeed * Time.deltaTime / circleRadius;

        float x = Mathf.Cos(angle) * circleRadius;
        float z = Mathf.Sin(angle) * circleRadius;
        leader.position = new Vector3(x, leader.position.y, z);

        leader.forward = new Vector3(-Mathf.Sin(angle), 0, Mathf.Cos(angle));
    }

    void MoveAgentToFormation(Transform agent, Vector3 offset)
    {
        Vector3 targetPosition = leader.position + leader.TransformDirection(offset);

        Vector3 currentVelocity = velocities[agent];
        agent.position = Vector3.SmoothDamp(agent.position, targetPosition, ref currentVelocity, smoothTime);
        velocities[agent] = currentVelocity;

        agent.forward = Vector3.Lerp(agent.forward, leader.forward, Time.deltaTime * 5f);
    }

    List<Vector3> CalculateVFormationOffsets()
    {
        List<Vector3> offsets = new List<Vector3>();
        int halfCount = agents.Count / 2;

        for (int i = 0; i < agents.Count; i++)
        {
            int wing = i % 2 == 0 ? 1 : -1;
            int index = Mathf.CeilToInt(i / 2f);

            float xOffset = index * wing * formationSpacing;
            float zOffset = -index * formationSpacing;

            offsets.Add(new Vector3(xOffset, 0, zOffset));
        }

        return offsets;
    }

    void OnDrawGizmos()
    {
        if (leader != null && formationOffsets != null)
        {
            foreach (var offset in formationOffsets)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(leader.position + leader.TransformDirection(offset), 0.2f);
            }
        }
    }
}
