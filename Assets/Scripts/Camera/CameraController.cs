using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    private Vector3 CamPos = new Vector3(-0.21f, 21.98f, -31.61f);

    // Update is called once per frame
    void Update()
    {
        float HorizontalInput = Input.GetAxis("CameraHorizontal");
        float VerticalInput = Input.GetAxis("CameraVertical");

        // Calculate movement direction based on input
        Vector3 moveDirection = new Vector3(HorizontalInput, 0f, VerticalInput).normalized;

        // Move the camera based on the calculated direction
        transform.Translate(moveDirection * 10f * Time.deltaTime);
    }
}
