using System.Collections;
using System.Collections.Generic;
using System.Linq;
//using System.Numerics;
using UnityEngine;

public class GradientDescent : MonoBehaviour
{
    [SerializeField] private GameObject Target;
    [SerializeField] private List<GameObject> Joints = new List<GameObject>();

    [SerializeField] private float SamplingDistance = 0.1f;
    [SerializeField] private float LearningRate = 1f;
    [SerializeField] private float DistanceThreshold = 0.1f;

    private float[] angles;

    private void Start()
    {
        angles = new float[Joints.Count];

        for (int i = 0; i < Joints.Count; i++)
        {
            if (Joints[i].GetComponent<RobotJoint>().Axis.x == 1)
            {
                angles[i] = Joints[i].GetComponent<Transform>().rotation.x;
            }
            else if (Joints[i].GetComponent<RobotJoint>().Axis.y == 1)
            {
                angles[i] = Joints[i].GetComponent<Transform>().rotation.y;
            }
            else if (Joints[i].GetComponent<RobotJoint>().Axis.z == 1)
            {
                angles[i] = Joints[i].GetComponent<Transform>().rotation.z;
            }
        }
    }

    private void Update()
    {
        InverseKinematics(Target.GetComponent<Transform>().position);
    }

    public Vector3 ForwardKinematics (float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;

        for (int jointNo = 1; jointNo < Joints.Count; jointNo++)
        {
            rotation *= Quaternion.AngleAxis(angles[jointNo], Joints[jointNo].GetComponent<RobotJoint>().Axis);
            Vector3 nextPoint = prevPoint + rotation * Joints[jointNo].GetComponent<RobotJoint>().StartOffset;

            prevPoint = nextPoint;
            Debug.Log(prevPoint);
        }

        //Debug.Log(prevPoint);
        return prevPoint;
    }

    public float DistanceFromTarget (Vector3 target)
    {
        Vector3 point = ForwardKinematics(angles);

        float distance = Vector3.Distance(point, target);

        Debug.Log(distance);
        return distance;
    }

    public float PartialGradient (Vector3 target, int angleNo)
    {
        float angle = angles[angleNo];

        float f_x = DistanceFromTarget(target);

        angles[angleNo] += SamplingDistance;
        float f_x_plus_d = DistanceFromTarget(target);

        float gradnient = (f_x_plus_d - f_x) / SamplingDistance;

        angles[angleNo] = angle;

        return gradnient;
    }

    public void InverseKinematics (Vector3 target)
    {
        for (int angleNo = 0; angleNo < Joints.Count; angleNo++)
        {
            float gradient = PartialGradient(target, angleNo);
            angles[angleNo] -= LearningRate * gradient;

            //angles[angleNo] = Mathf.Clamp(angles[angleNo], Joints[angleNo].GetComponent<RobotJoint>().minAngle, Joints[angleNo].GetComponent<RobotJoint>().maxAngle);

            Debug.Log("angle " + angleNo + " : " + angles[angleNo]);

            //for (int i = 0; i < Joints.Count; i++)
            //{
            //    if (Joints[i].GetComponent<RobotJoint>().Axis.x > 0.5)
            //    {
            //        Joints[i].GetComponent<Transform>().rotation = Quaternion.
            //    }
            //    else if (Joints[i].GetComponent<RobotJoint>().Axis.y > 0.5)
            //    {
            //        Joints[i].GetComponent<RobotJoint>().Axis.y = angles[i];
            //    }
            //    else if (Joints[i].GetComponent<RobotJoint>().Axis.z > 0.5)
            //    {
            //        Joints[i].GetComponent<RobotJoint>().Axis.z = angles[i];
            //    }
            //}

            for (int i = 0; i < Joints.Count; i++)
            {
                Joints[i].GetComponent<Transform>().localRotation = Quaternion.AngleAxis(angles[i], Joints[i].GetComponent<RobotJoint>().Axis);
            }

            //Updating LearningRate and SamplingDistance
            float head2TargetDistance = Vector3.Distance(Joints[Joints.Count - 1].transform.position, target);
            SamplingDistance = head2TargetDistance / 2;
            LearningRate = head2TargetDistance * 7;

            //if (DistanceFromTarget(target, angles) < DistanceThreshold)
            //    return;
        }
    }
}
