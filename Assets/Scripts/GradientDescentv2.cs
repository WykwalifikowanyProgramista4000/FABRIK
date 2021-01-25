using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
//using System.Numerics;
using UnityEngine;

public class GradientDescentv2 : MonoBehaviour
{
    [SerializeField] private string ChainName = "uassigned";

    [SerializeField] private GameObject Target;
    [SerializeField] private List<GameObject> Joints = new List<GameObject>();

    [SerializeField] private float SamplingDistance = 0.1f;
    [SerializeField] private float LearningRate = 1f;
    [SerializeField] private float DistanceThreshold = 0.1f;

    private float[][] angles;

    // Diagnostics
    private int iterationsXD;
    Stopwatch stopWatch = new Stopwatch();

    private void Start()
    {
        angles = new float[Joints.Count][];

        for (int i = 0; i < Joints.Count; i++)
        {
            angles[i] = new float[3] { Joints[i].GetComponent<Transform>().rotation.x, Joints[i].GetComponent<Transform>().rotation.y, Joints[i].GetComponent<Transform>().rotation.z };
        }

        //TEST
        iterationsXD = 0;
        stopWatch.Reset();
        //stopWatch.Start();
        //TEST
    }

    void TEST()
    {
        iterationsXD++;

        //UnityEngine.Debug.Log("hehe: " + iterationsXD);

        //if (iterationsXD % 50 == 0 || iterationsXD == 283 || (iterationsXD % 10 == 0 && iterationsXD<50))
        //{
        //    Debug.Break();
        //}

        if (iterationsXD == 283)
        {
            stopWatch.Stop();

            UnityEngine.Debug.Log(ChainName + " - iterations: " + iterationsXD + " | time elapsed: " + stopWatch.Elapsed);
            
            UnityEngine.Debug.Break();
        }
    }


    private void Update()
    {
        stopWatch.Start();
        InverseKinematics(Target.GetComponent<Transform>().position);
        stopWatch.Stop();

        TEST();
    }

    public Vector3 ForwardKinematics (float[][] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;

        for (int jointNo = 1; jointNo < Joints.Count; jointNo++)
        {
            rotation *= Quaternion.AngleAxis(angles[jointNo][0], new Vector3(1,0,0));
            rotation *= Quaternion.AngleAxis(angles[jointNo][1], new Vector3(0,1,0));
            rotation *= Quaternion.AngleAxis(angles[jointNo][2], new Vector3(0,0,1));
            Vector3 nextPoint = prevPoint + rotation * Joints[jointNo].GetComponent<RobotJoint>().StartOffset;

            prevPoint = nextPoint;
            //Debug.Log(prevPoint);
        }

        //Debug.Log(prevPoint);
        return prevPoint;
    }

    public float DistanceFromTarget (Vector3 target)
    {
        Vector3 point = ForwardKinematics(angles);

        float distance = Vector3.Distance(point, target);

        //Debug.Log(distance);
        return distance;
    }

    public float PartialGradient (Vector3 target, int angleNo, int axisId)
    {
        float angle = angles[angleNo][axisId];

        float f_x = DistanceFromTarget(target);

        angles[angleNo][axisId] += SamplingDistance;
        float f_x_plus_d = DistanceFromTarget(target);

        float gradnient = (f_x_plus_d - f_x) / SamplingDistance;

        angles[angleNo][axisId] = angle;

        return gradnient;
    }

    public void InverseKinematics (Vector3 target)
    {
        for (int angleNo = 0; angleNo < Joints.Count; angleNo++)
        {
            for (int axisId = 0; axisId <= 2; axisId++)
            {
                float gradient = PartialGradient(target, angleNo, axisId);
                angles[angleNo][axisId] -= LearningRate * gradient;
            }
            

            #region debug
            //angles[angleNo] = Mathf.Clamp(angles[angleNo], Joints[angleNo].GetComponent<RobotJoint>().minAngle, Joints[angleNo].GetComponent<RobotJoint>().maxAngle);

            //Debug.Log("angle " + angleNo + " : " + angles[angleNo]);

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
            #endregion

            for (int i = 0; i < Joints.Count; i++)
            {
                Joints[i].GetComponent<Transform>().localRotation = Quaternion.AngleAxis(angles[i][0], new Vector3(1, 0, 0)) *
                                                                    Quaternion.AngleAxis(angles[i][1], new Vector3(0, 1, 0)) *
                                                                    Quaternion.AngleAxis(angles[i][2], new Vector3(0, 0, 1));
            }

            //Updating LearningRate and SamplingDistance
            float head2TargetDistance = Vector3.Distance(Joints[Joints.Count - 1].transform.position, target);

            //if (DistanceFromTarget(target, angles) < DistanceThreshold)
            //    return;
        }
    }
}
