using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class AgentControls_NoObstacles : Agent
{
    [SerializeField] private Transform Target;
    [SerializeField] private RayPerceptionSensorComponent3D sensor;

    [SerializeField] private int MovementMode = 0;

    private Material enviromentMaterial;
    public GameObject enviroment;

    private float PreviousDistance = 0;
    private float TimeTaken = 0;

    //Controls and movement
    public float MoveSpeed = 4f;

    public override void Initialize()
    {
        enviromentMaterial = enviroment.GetComponent<Renderer>().material;
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("Begin Episode! - ");

        if (MovementMode == 1)
        {
            // 1D Movement
            transform.localPosition = new Vector3(0, 0.3f, 0);

            int rand = Random.Range(0, 2);

            if (rand == 0)
            {
                Target.localPosition = new Vector3(5, 0, 0);
            }
            else
            {
                Target.localPosition = new Vector3(-5, 0, 0);
            }
        }
        else if (MovementMode == 2)
        {
            // Set Agent Position - 2D Movement
            transform.localPosition = new Vector3(Random.Range(-6, 6), 0.5f, Random.Range(-3, 3));

            Target.localPosition = new Vector3(Random.Range(-6, 6), 0.5f, Random.Range(-3, 3));
        }
        else
        {
            // Set Agent Position - 3D Movement
            transform.localPosition = new Vector3(Random.Range(-6, 6), Random.Range(0, 8), Random.Range(-3, 3));

            Target.localPosition = new Vector3(Random.Range(-6, 6), Random.Range(0, 8), Random.Range(-3, 3));
        }
    }

    void Update()
    {

        TimeTaken += Time.deltaTime;
    }

    public override void CollectObservations(VectorSensor Sensor)
    {
        if (MovementMode == 1 || MovementMode == 2)
        {
            // position of Agent - 3 values
            Sensor.AddObservation(transform.localPosition);
            //Position of target
            Sensor.AddObservation(Target.localPosition);
        }
        else
        {
            // position of Agent - 3 values
            Sensor.AddObservation(transform.localPosition);
            // Forward of Agent - 3 values
            Sensor.AddObservation(transform.forward);
            // Target Location
            // add distance to target - 1 value
            Sensor.AddObservation(Vector3.Distance(transform.localPosition, Target.localPosition));
            // add direction to target - 3 values
            Sensor.AddObservation((Target.transform.localPosition - transform.localPosition).normalized);
            // Observe the local rotation
            Sensor.AddObservation(transform.localRotation.normalized);

            var PerceptionInput = sensor.GetRayPerceptionInput();
            var rayOutputs = RayPerceptionSensor.Perceive(PerceptionInput, true).RayOutputs;

            int lengthOfRayOutputs = rayOutputs.Length;

            for (int i = 0; i < lengthOfRayOutputs; i++)
            {
                GameObject goHit = rayOutputs[i].HitGameObject;
                if (goHit != null)
                {
                    var rayDirection = rayOutputs[i].EndPositionWorld - rayOutputs[i].StartPositionWorld;
                    var scaledRayLength = rayDirection.magnitude;
                    float rayHitDistance = rayOutputs[i].HitFraction * scaledRayLength;

                    if (goHit.tag == "Target")
                    {
                        Debug.Log("Target!");
                        // add target location
                        //Sensor.AddObservation(Target.localPosition);
                        //AddReward(0.05f);
                    }
                }
            }

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
        
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (MovementMode == 1)
        {
            float Forward = actions.ContinuousActions[0];

            // Movement System

            Vector3 Velocity = new Vector3(Forward, 0, 0);
            Velocity = Velocity.normalized * Time.deltaTime * MoveSpeed;

            transform.localPosition += Velocity;
        }
        else if (MovementMode == 2)
        {
            float Forward = actions.ContinuousActions[0];
            float Left = actions.ContinuousActions[1];

            // Movement System

            Vector3 Velocity = new Vector3(Forward, 0, Left);
            Velocity = Velocity.normalized * Time.deltaTime * MoveSpeed;

            transform.localPosition += Velocity;
        }
        else
        {
            float Forward = actions.ContinuousActions[0];
            float Left = actions.ContinuousActions[1];
            float Up = actions.ContinuousActions[2];

            // Movement System

            Vector3 Velocity = new Vector3(Left, Up, Forward);
            Velocity = Velocity.normalized * Time.deltaTime * MoveSpeed;

            transform.localPosition += Velocity;
        }
        

        // raycast check!

        // reward for every step taken towards target.
        float distancetotarget = Vector3.Distance(transform.localPosition, Target.localPosition);

        AddReward(0.01f * (PreviousDistance - distancetotarget));

        PreviousDistance = distancetotarget;
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

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Bounds")
        {
            AddReward(-15f);
            enviromentMaterial.color = Color.red;
            EndEpisode();
        }

        if (other.gameObject.tag == "Target")
        {
            float efficiency = Mathf.Clamp(2f - TimeTaken, 0f, 2f);
            AddReward(5f + efficiency);
            enviromentMaterial.color = Color.green;
            EndEpisode();
        }
    }
}

