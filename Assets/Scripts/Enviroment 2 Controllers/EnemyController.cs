using System;
using System.Collections;
using System.Collections.Generic;
using Assets.Scripts;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;
using Random = UnityEngine.Random;

public class EnemyController : Agent
{
    // constants
    private Constants constant = new Constants();
    // Distance Value
    private float PreviousDistance = 0;
    [SerializeField] private DistanceTypes DistanceType = DistanceTypes.Euclidean;

    [SerializeField] public GameObject Agent;
    public AgentControls AgentControls;

    // Timer values
    [SerializeField] private int EpisodeLength;
    private float TimeLeft;
    private float TimeTaken = 0;

    //Controls and movement
    public float MoveSpeed = 4f;

    // For Obstacle/enviromental related variables.
    [SerializeField] private List<GameObject> Obstacles = new List<GameObject>();
    private Material enviromentMaterial;
    public GameObject enviroment;

    public override void Initialize()
    {
        enviromentMaterial = enviroment.GetComponent<Renderer>().material;
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log("Begin Episode! - ");

        // Set Enemy Agent Position - 3D Movement
        Vector3 SpawnPos = new Vector3(Random.Range(-9, 9), Random.Range(0, 11), Random.Range(-9, 9));

        while (!constant.CheckOverlaps(Agent.transform.localPosition, SpawnPos, 5f))
        {
            SpawnPos = new Vector3(Random.Range(-9, 9), Random.Range(0, 11), Random.Range(-9, 9));
        }

        transform.localPosition = SpawnPos;

        EpisodeTimer();
    }

    void Update()
    {
        CheckTime();

        TimeTaken += Time.deltaTime;

        if (Vector3.Distance(Agent.transform.localPosition, transform.localPosition) >= 30)
        {
            EndEpisode(); //Out of bounds
        }
    }

    public override void CollectObservations(VectorSensor Sensor)
    {
        // position of Enemy Agent - 3 values
        Sensor.AddObservation(transform.localPosition);
        // Forward of Enemy Agent - 3 values
        Sensor.AddObservation(transform.forward);

        // Give Enemy Euclidean Distance
        Sensor.AddObservation(Vector3.Distance(transform.localPosition, Agent.transform.localPosition));

        // add direction to target - 3 values
        Sensor.AddObservation((Agent.transform.localPosition - transform.localPosition).normalized);

        // Raycast to detect what is above the agent.
        float RaycastDistance = 10f;

        RaycastHit Hit;
        if (Physics.Raycast(transform.localPosition, transform.up, out Hit, RaycastDistance))
        {
            float obstacleDistance = Hit.distance;
            Sensor.AddObservation(obstacleDistance); // add distance from obstacle
        }
        else
        {
            Sensor.AddObservation(RaycastDistance); // add max distance
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float Forward = actions.ContinuousActions[0];
        float Left = actions.ContinuousActions[1];
        float Up = actions.ContinuousActions[2];
        float pitch = actions.ContinuousActions[3];
        float yaw = actions.ContinuousActions[4];

        Vector3 Velocity = new Vector3(Left, Up, Forward);
        Velocity = Velocity.normalized * Time.deltaTime * MoveSpeed;

        transform.localPosition += Velocity;

        // raycast check!

        // reward for every step taken towards target.
        float distancetotarget = Vector3.Distance(transform.localPosition, Agent.transform.localPosition);

        AddReward(0.01f * (PreviousDistance - distancetotarget));

        PreviousDistance = distancetotarget;

        // punish for every step
        if (MaxStep > 0)
        {
            AddReward(-1f / MaxStep);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> ContinuousActions = actionsOut.ContinuousActions;
        ContinuousActions[0] = Input.GetAxisRaw("Forward/Backward");
        ContinuousActions[1] = Input.GetAxisRaw("Left/Right");
        ContinuousActions[2] = Input.GetAxisRaw("Up");
        ContinuousActions[3] = Input.GetAxisRaw("Pitch");
        ContinuousActions[4] = Input.GetAxisRaw("Look");
    }

    void EpisodeTimer()
    {
        TimeLeft = Time.time + EpisodeLength;
    }

    void CheckTime()
    {
        if (Time.time > TimeLeft)
        {
            enviromentMaterial.color = Color.blue;
            AddReward(-10f);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Bounds")
        {
            AddReward(-15f);
            enviromentMaterial.color = Color.red;
            AgentControls.EndEpisode();
            EndEpisode();
        }

        if (other.gameObject.tag == "Agent")
        {
            float efficiency = Mathf.Clamp(5f - TimeTaken, 0f, 5f);
            AddReward(15f + efficiency);
            enviromentMaterial.color = Color.cyan;
            AgentControls.AddReward(-10f);
            AgentControls.EndEpisode();
            EndEpisode();
        }

    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Obstacle")
        {
            AddReward(-5);
            enviromentMaterial.color = Color.yellow;
        }
    }
}
