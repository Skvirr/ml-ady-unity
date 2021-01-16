// using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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

public class CarAgent : Agent
{
    // Car
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    [Range(0, 1f)] public float areaPercentage;
    // ML Agents
    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;
    [SerializeField] private Rigidbody vehicleRigidBody;

    public override void OnEpisodeBegin()
    {
        ResetVehicle();

        float radius = 45f;
        radius = radius * areaPercentage;

        transform.localPosition = new Vector3(0, 0, radius);
        transform.rotation = Quaternion.Euler(0, 180, 0);
        // transform.rotation = Quaternion.Euler(0, Random.Range(0, 359f), 0);

        targetTransform.localPosition = transform.localPosition + new Vector3(Random.Range(-radius, radius), 0, Random.Range(-radius * 1, -radius * 2));
        // targetTransform.localPosition = new Vector3(Random.Range(-randSize, randSize), 0.5f, Random.Range(-randSize, randSize));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativePosition = targetTransform.localPosition - transform.localPosition;

        Quaternion rotation = transform.rotation;
        relativePosition = Quaternion.Inverse(rotation) * relativePosition;

        sensor.AddObservation(relativePosition.x);
        sensor.AddObservation(relativePosition.y);
        sensor.AddObservation(relativePosition.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float motor = maxMotorTorque * actions.ContinuousActions[0];
        float steering = maxSteeringAngle * actions.ContinuousActions[1];
        int brake = actions.DiscreteActions[0];
        // int brake = 0;

        motor = motor <= 0 ? motor * 0.75f : maxMotorTorque;

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
                axleInfo.leftWheel.brakeTorque = brake == 1 ? maxMotorTorque * 2.5f : 0;
                axleInfo.rightWheel.brakeTorque = brake == 1 ? maxMotorTorque * 2.5f : 0;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }

        AddReward(-0.02f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
        continousActions[0] = Input.GetAxisRaw("Vertical");
        continousActions[1] = Input.GetAxisRaw("Horizontal");
        discreteActions[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

    private void ResetVehicle()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = 0;
                axleInfo.rightWheel.steerAngle = 0;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = 0;
                axleInfo.rightWheel.motorTorque = 0;
                axleInfo.leftWheel.brakeTorque = 0;
                axleInfo.rightWheel.brakeTorque = 0;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }

        vehicleRigidBody.velocity = Vector3.zero;
        vehicleRigidBody.angularVelocity = Vector3.zero;
        vehicleRigidBody.Sleep();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (vehicleRigidBody.velocity.x > 0.1f & vehicleRigidBody.velocity.z > 0.1f)
        {
            return;
        }

        if (other.TryGetComponent<Goal>(out Goal goal))
        {
            AddReward(+20f * (1f - 0.075f * (vehicleRigidBody.velocity.magnitude)));
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        // https://docs.unity3d.com/ScriptReference/WheelCollider.ConfigureVehicleSubsteps.html
        collider.ConfigureVehicleSubsteps(5f, 5, 15);

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
}