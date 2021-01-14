using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarAgent : MonoBehaviour
{
    void Start()
    {

    }

    public float MotorForce, SteerForce, BreakForce, friction;
    public WheelCollider FLW, FRW, RLW, RRW;
    public GameObject car;

    void Update()
    {
        float v = Input.GetAxis("Vertical") * MotorForce;

        RLW.motorTorque = v;
        RRW.motorTorque = v;

        car.transform.Rotate(Vector3.up * SteerForce * Time.deltaTime * Input.GetAxis("Horizontal"), Space.World);

        if (Input.GetKey(KeyCode.Space))
        {
            RLW.brakeTorque = BreakForce;
            RRW.brakeTorque = BreakForce;
        }
        if (Input.GetKeyUp(KeyCode.Space))
        {
            RLW.brakeTorque = 0;
            RRW.brakeTorque = 0;
        }
        if (Input.GetAxis("Vertical") == 0)
        {
            if (RLW.brakeTorque <= BreakForce && RRW.brakeTorque <= BreakForce)
            {
                RLW.brakeTorque += friction * Time.deltaTime * BreakForce;
                RRW.brakeTorque += friction * Time.deltaTime * BreakForce;
            }
            else
            {
                RLW.brakeTorque = BreakForce;
                RRW.brakeTorque = BreakForce;
            }
        }
        else
        {
            RLW.brakeTorque = 0;
            RRW.brakeTorque = 0;
        }


    }
}
