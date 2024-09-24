using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class Agent_Obstacles : Agent
{
    [SerializeField] private Transform Target;
    [SerializeField] private RayPerceptionSensorComponent3D sensor;
    [SerializeField] private int MinkowskiValue = 4;
    private int RewardCount = 0;

    public enum DistanceTypes
    {
        Euclidean,
        Manhattan,
        Minkowski,
        Chebyshev
    }
    [SerializeField] private DistanceTypes DistanceType = DistanceTypes.Euclidean;

    // For Obstacle related variables.
    [SerializeField] private List<GameObject> Obstacles = new List<GameObject>();

    // Setup Enviroment
    private Material enviromentMaterial;
    public GameObject enviroment;
    // Timer and float for calculating how whether agent gets reward for moving closer to target.
    private float PreviousDistance = 0;
    private float TimeTaken = 0;

    //Controls and movement
    public float MoveSpeed = 4f;

    // Timer values
    [SerializeField] private int EpisodeLength;
    private float TimeLeft;

    // Test values
    private int CollisionCount = 0;
    private int WinCount = 0;
    private int LossCount = 0;
    private int TotalEpisodes = 0;

    public override void Initialize()
    {
        enviromentMaterial = enviroment.GetComponent<Renderer>().material;
    }

    bool CheckOverlaps(Vector3 ObjectPosition, Vector3 ExistingObjectPosition, float MinimumDistance)
    {
        // check two objects are further away than minimum distance.
        float ObjectDistance = Vector3.Distance(ObjectPosition, ExistingObjectPosition);
        if (MinimumDistance <= ObjectDistance)
        {
            return true;
        }

        return false;
    }

    public override void OnEpisodeBegin()
    {
        // Increment episode number
        TotalEpisodes++;
        // Set Agent Position - 3D Movement - Enviroment 2
        Vector3 AgentPosition = new Vector3(Random.Range(-9, 9), Random.Range(0, 11), Random.Range(-9, 9));
        // Check for overlap with obstacles before placing
        int counter = 0;
        for (int i = 0; i < Obstacles.Count; i++)
        {
            if (counter <= 10)
            {
                if (!CheckOverlaps(AgentPosition, Obstacles[i].transform.position, 2f))
                {
                    AgentPosition = new Vector3(Random.Range(-9, 9), Random.Range(0, 11), Random.Range(-9, 9));
                    i--;
                }

                counter++;


            }
            else
            {
                break;
            }
        }
        transform.localPosition = AgentPosition;

        // Set Target Position
        Vector3 TargetPosition = new Vector3(Random.Range(-9, 9), Random.Range(0.5f, 11), Random.Range(-9, 9));
        counter = 0;
        // Check for overlap with obstacles and agent before placing
        for (int i = 0; i < Obstacles.Count; i++)
        {
            if (counter <= 10)
            {
                if (!CheckOverlaps(TargetPosition, Obstacles[i].transform.position, 3f))
                {
                    TargetPosition = new Vector3(Random.Range(-9, 9), Random.Range(0, 11), Random.Range(-9, 9));
                    i--;
                }

                if (!CheckOverlaps(TargetPosition, AgentPosition, 1f))
                {
                    TargetPosition = new Vector3(Random.Range(-9, 9), Random.Range(0.5f, 11), Random.Range(-9, 9));
                    //Debug.Log("Overlap! Regenerating...");
                }

                counter++;


            }
            else
            {
                break;
            }
        }

        Target.localPosition = TargetPosition;


        foreach (GameObject obstacle in Obstacles)
        {
            Obstacles ObstacleScript = obstacle.GetComponent<Obstacles>();
            if (ObstacleScript != null)
            {
                ObstacleScript.ResetObstacle();
            }
        }

        // start timer again
        EpisodeTimer();
        RewardCount = 0;
    }

    void Update()
    {
        CheckTime();

        TimeTaken += Time.deltaTime;

        // if agent falls out of bounds
        if (Vector3.Distance(Target.position, transform.position) >= 30)
        {
            EndEpisode(); //Out of bounds
        }
    }

    public override void CollectObservations(VectorSensor Sensor)
    {
        // position of Agent - 3 values
        Sensor.AddObservation(transform.localPosition);
        // Forward of Agent - 3 values
        Sensor.AddObservation(transform.forward);

        // Call function to calculate distance to target - 1 value
        float Distance = CalculateDistance();
        Sensor.AddObservation(Distance);

        // add direction to target - 3 values
        Sensor.AddObservation((Target.transform.localPosition - transform.localPosition).normalized);

        // Observe the local rotation
        Sensor.AddObservation(transform.localRotation.normalized);

        var PerceptionInput = sensor.GetRayPerceptionInput();
        var rayOutputs = RayPerceptionSensor.Perceive(PerceptionInput, true).RayOutputs;

        int lengthOfRayOutputs = rayOutputs.Length;

        bool TargetSighted = false;
        for (int i = 0; i < lengthOfRayOutputs; i++)
        {
            GameObject goHit = rayOutputs[i].HitGameObject;
            if (goHit != null)
            {
                var rayDirection = rayOutputs[i].EndPositionWorld - rayOutputs[i].StartPositionWorld;
                var scaledRayLength = rayDirection.magnitude;
                float rayHitDistance = rayOutputs[i].HitFraction * scaledRayLength;

                //Sensor.AddObservation(scaledRayLength);

                if (goHit.tag == "Target")
                {
                    TargetSighted = true;
                }
            }
        }



        if (TargetSighted)
        {
            if (RewardCount >= 0.5)
            {
                AddReward(0.05f);
            }
            else
            {
                TargetSighted = false;
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

    private float CalculateDistance()
    {
        switch (DistanceType)
        {
            case DistanceTypes.Euclidean:
                // Euclidian Distance Calculation
                //Debug.Log("Returned Euclidean");
                return Vector3.Distance(transform.localPosition, Target.localPosition);

            case DistanceTypes.Manhattan:
                // Manhattan Distance Calculation
                //Debug.Log("Returned Manhattan");
                return CalculateManhattan();

            case DistanceTypes.Minkowski:
                //Debug.Log("Returned Minkowski");
                return CalculateMinkowski(MinkowskiValue);

            case DistanceTypes.Chebyshev:
                // Chebyshev distance calculation
                //Debug.Log("Returned Chebyshev");
                return CalculateChebyshev();
            default:
                //Debug.Log("Returned -1");
                return -1;
                break;
        }
    }

    private float CalculateChebyshev()
    {
        // point 1 = agent point
        // point 2 = target point
        // Calculate the absolute differences along each dimension
        float deltaX = Mathf.Abs(transform.localPosition.x - Target.localPosition.x);
        float deltaY = Mathf.Abs(transform.localPosition.y - Target.localPosition.y);
        float deltaZ = Mathf.Abs(transform.localPosition.z - Target.localPosition.z);

        // Find the maximum absolute difference among all dimensions
        return Mathf.Max(deltaX, deltaY, deltaZ);
    }

    private float CalculateMinkowski(float p)
    {
        // point 1 = agent point
        // point 2 = target point
        // Calculate the absolute differences along each dimension
        float deltaX = Mathf.Abs(transform.localPosition.x - Target.localPosition.x);
        float deltaY = Mathf.Abs(transform.localPosition.y - Target.localPosition.y);
        float deltaZ = Mathf.Abs(transform.localPosition.z - Target.localPosition.z);

        // Sum the absolute differences raised to the power of p
        float sum = Mathf.Pow(deltaX, p) + Mathf.Pow(deltaY, p) + Mathf.Pow(deltaZ, p);

        // Take the pth root of the sum
        return Mathf.Pow(sum, 1.0f / p);
    }

    private float CalculateManhattan()
    {
        // point 1 = agent point
        // point 2 = target point
        // Calculate the absolute differences along each dimension
        float deltaX = Mathf.Abs(transform.localPosition.x - Target.localPosition.x);
        float deltaY = Mathf.Abs(transform.localPosition.y - Target.localPosition.y);
        float deltaZ = Mathf.Abs(transform.localPosition.z - Target.localPosition.z);

        // Find the maximum absolute difference among all dimensions
        return deltaX + deltaY + deltaZ;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float Forward = actions.ContinuousActions[0];
        float Left = actions.ContinuousActions[1];
        float Up = actions.ContinuousActions[2];
        float pitch = actions.ContinuousActions[3];
        float yaw = actions.ContinuousActions[4];

        // Movement System

        Vector3 Velocity = new Vector3(Left, Up, Forward);
        Velocity = Velocity.normalized * Time.deltaTime * MoveSpeed;

        transform.localPosition += Velocity;

        // raycast check!

        // reward for every step taken towards target.
        float distancetotarget = Vector3.Distance(transform.localPosition, Target.localPosition);

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
        // Called to get a time for episode to end
        TimeLeft = Time.time + EpisodeLength;
    }

    void CheckTime()
    {
        // punishes if time limit is reached
        if (Time.time > TimeLeft)
        {
            enviromentMaterial.color = Color.blue;
            AddReward(-10f);
            LossCount++;
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Bounds")
        {
            AddReward(-15f);
            enviromentMaterial.color = Color.red;
            LossCount++;
            EndEpisode();
        }

        if (other.gameObject.tag == "Target")
        {
            float efficiency = Mathf.Clamp(5f - TimeTaken, 0f, 5f);
            AddReward(15f + efficiency);
            enviromentMaterial.color = Color.green;
            WinCount++;
            EndEpisode();
        }

    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Obstacle")
        {
            AddReward(-5);
            CollisionCount++;
            enviromentMaterial.color = Color.yellow;
        }
    }

    private void OnDestroy()
    {
        Debug.Log("Training Over!");
        Debug.Log("Collisions - " + CollisionCount);
        Debug.Log("Wins - " + WinCount);
        Debug.Log("Losses - " + LossCount);
        Debug.Log("Total Episodes - " + TotalEpisodes);
    }
}
