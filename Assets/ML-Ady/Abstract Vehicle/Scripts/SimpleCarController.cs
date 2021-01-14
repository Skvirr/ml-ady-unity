using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class SimpleCarController : MonoBehaviour
{
    // Car
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;

    // ML Agents
    // [SerializeField] private Material winMaterial;
    // [SerializeField] private Material loseMaterial;
    // [SerializeField] private MeshRenderer floorMeshRenderer;

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        // https://docs.unity3d.com/ScriptReference/WheelCollider.ConfigureVehicleSubsteps.html
        collider.ConfigureVehicleSubsteps(5f, 10, 15);

        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation * Quaternion.Euler(0, 0, 90f);
    }

    public void FixedUpdate()
    {
        // Debug.Log(Input.GetAxis("Vertical"));
        // Debug.Log(Input.GetAxis("Horizontal"));
        float motor = maxMotorTorque * (Input.GetAxis("Vertical"));
        motor = motor < 0 ? motor * 0.75f : motor;
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
                axleInfo.leftWheel.brakeTorque = Input.GetKey(KeyCode.Space) ? maxMotorTorque * 5 : 0;
                axleInfo.rightWheel.brakeTorque = Input.GetKey(KeyCode.Space) ? maxMotorTorque * 5 : 0;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }
}