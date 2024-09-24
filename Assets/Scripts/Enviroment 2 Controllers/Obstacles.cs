using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class Obstacles : MonoBehaviour
{
    // Final transform of obstacles, will go from starting to this and back
     [SerializeField] private Transform TargetTransform;
    // // 1 = position change 2 = rotation
     [SerializeField] private int ObstacleType;
     [SerializeField] private Transform StartingTransform;
     private Vector3 OriginPos;
     private Vector3 RandomRotation;
     private bool ToPosition = true;

    public void ResetObstacle()
    {
        // Function called on each new episode begin
        transform.rotation = StartingTransform.rotation;
        transform.position = StartingTransform.position;

        // calculate random rotation for rotating obstacle
        RandomRotation = CalcRandomRotation();
    }

    void Start()
    {
        ResetObstacle();

        OriginPos = StartingTransform.localPosition;
    }

    private Vector3 CalcRandomRotation()
    {
        // Random Rotation
        RandomRotation.x = Random.Range(0, 360);
        RandomRotation.y = 0;
        RandomRotation.z = Random.Range(0, 360);

        return RandomRotation;
    }

    // Update is called once per frame
    void Update()
    {
        switch (ObstacleType)
        {
            case 1:
                //ObstacleMovement();
                break;
            case 2:
                //ObstacleRotation();
                break;
            default:
                break;
        }
    }

    private void ObstacleRotation()
    {
        // rotate rotating obstacles
        transform.Rotate(RandomRotation * Time.deltaTime / 10);
    }

    private void ObstacleMovement()
    {
        if (ToPosition) // If moving to the target position
        {
            // apply movement to moving obstacles
            float step = 5f * Time.deltaTime;
            transform.localPosition = Vector3.MoveTowards(transform.localPosition, new Vector3(transform.localPosition.x, TargetTransform.localPosition.y, transform.localPosition.z), step);

            // Check if the obstacle has reached the target position
            if (Mathf.Approximately(transform.localPosition.y, TargetTransform.localPosition.y))
            {
                // Once reached, set bool to move back
                ToPosition = false;
               // Debug.Log("Position Changed to false");
            }
        }
        else // moving back to start position
        {
            // apply movement to moving obstacles
            float step = 5f * Time.deltaTime;
            transform.localPosition = Vector3.MoveTowards(transform.localPosition, OriginPos, step);

            // Check if the obstacle has reached the target position
            if (Mathf.Approximately(transform.localPosition.y, OriginPos.y))
            {
                // Once reached, set bool to move back
                ToPosition = true;
                //Debug.Log("Position Changed to true");
            }
        }
        
    }
}
