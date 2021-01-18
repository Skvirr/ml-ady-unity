// using System.Runtime;
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
    public GameObject target;
    public Material winMaterial;
    public Material loseMaterial;
    public MeshRenderer floorMeshRenderer;
    public Material winGround;
    public Material loseGround;
    public Rigidbody vehicleRigidbody;

    private void Start()
    {
        vehicleRigidbody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        ResetVehicle();

        float radius = 45f;
        radius = radius * areaPercentage;

        transform.localPosition = new Vector3(
            0f * radius + Random.Range(-radius * 0.25f, radius * 0.25f),
            0,
            0.9f * radius + Random.Range(-radius * 0.1f, radius * 0.1f)
        );
        transform.rotation = Quaternion.Euler(
            0,
            180 + Random.Range(-180 * areaPercentage, 180 * areaPercentage),
            0
        );
        target.transform.localPosition = new Vector3(
            transform.localPosition.x + Random.Range(-radius, radius),
            20f,
            transform.localPosition.z + Random.Range(-radius * 0, -radius * 2)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativePosition = target.transform.localPosition - transform.localPosition;

        Quaternion rotation = transform.rotation;
        relativePosition = Quaternion.Inverse(rotation) * relativePosition;

        sensor.AddObservation(relativePosition.x);
        sensor.AddObservation(relativePosition.y);
        sensor.AddObservation(relativePosition.z);

        Vector3 gravityDirection = (transform.rotation * Vector3.down).normalized;

        sensor.AddObservation(gravityDirection.x);
        sensor.AddObservation(gravityDirection.y);
        sensor.AddObservation(gravityDirection.z);
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

        vehicleRigidbody.velocity = Vector3.zero;
        vehicleRigidbody.angularVelocity = Vector3.zero;
        vehicleRigidbody.Sleep();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (vehicleRigidbody.velocity.x > 0.1f & vehicleRigidbody.velocity.z > 0.1f)
        {
            return;
        }

        if (other.TryGetComponent<TrashSpawner>(out TrashSpawner trashSpawner))
        {
            AddReward(+20f * (1f - 0.075f * (vehicleRigidbody.velocity.magnitude)));
            floorMeshRenderer.material = winGround;
            EndEpisode();
        }
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(-1f);
            floorMeshRenderer.material = loseGround;
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