using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.UIElements;

public class CCD : MonoBehaviour
{
    [SerializeField] private string ChainName = "unnamed CCD";

    [Header("--- Components ---")]
    [SerializeField] private GameObject m_TargetPoint;
    [SerializeField] private GameObject m_Base;
    [SerializeField] private List<GameObject> m_Joints = new List<GameObject>();

    [Header("--- Settings ---")]
    [SerializeField] private int m_MaxCCDIterations = 20;
    [SerializeField] private float m_DistanceFromTargetTolerance = 0.01f;

    [Header("# DEBGUG #")]
    [SerializeField] private GameObject debug_TestPoint;
    [SerializeField] private float debug_RayDuration = 0.1f;

    private float[] _cosTheta;
    private Vector3[] _rotationAxis;

    //TEST
    private int iterationsXD;
    Stopwatch stopWatch = new Stopwatch();

    void TEST()
    {
        iterationsXD++;

        if (iterationsXD == 0 || iterationsXD == 1 || iterationsXD == 2 || iterationsXD == 3 || iterationsXD == 10 || iterationsXD == 37)
        {
            //stopWatch.Stop();
            float distance = Vector3.Distance(m_Joints[m_Joints.Count - 1].transform.position, m_TargetPoint.transform.position);
            UnityEngine.Debug.Log(ChainName + " - iterations: " + iterationsXD + " | distance: " + distance + " | time elapsed: " + stopWatch.Elapsed);

            UnityEngine.Debug.Break();
            //stopWatch.Start();
        }
    }

    private void Start()
    {
        _cosTheta = new float[m_Joints.Count - 1];
        _rotationAxis = new Vector3[m_Joints.Count];

        //TEST
        iterationsXD = 0;
        stopWatch.Reset();
        //stopWatch.Start();
        //TEST
    }

    private void Update()
    {
        stopWatch.Start();
        SolveCCD();
        UnityEngine.Debug.Break();

        //TEST();
        stopWatch.Stop();
    }

    private void SolveCCD()
    {
        Vector3 tempLeftSide;
        Vector3 tempRightSide;

        float tempLeftNormal;
        float tempRightNormal;

        //max error


        for(int jointNo = m_Joints.Count-2; jointNo >=0; jointNo--)
        {
            tempLeftSide = (m_Joints[m_Joints.Count - 1].transform.position - m_Joints[jointNo].transform.position);
            tempRightSide = (m_TargetPoint.transform.position - m_Joints[jointNo].transform.position);

            tempLeftNormal = Vector3.Magnitude(tempLeftSide);
            tempRightNormal = Vector3.Magnitude(tempRightSide);

            _cosTheta[jointNo] = Vector3.Dot((tempLeftSide / tempLeftNormal), (tempRightSide / tempRightNormal));
            _rotationAxis[jointNo] = Vector3.Cross((tempLeftSide / tempLeftNormal), (tempRightSide / tempRightNormal));

            float degrees = Mathf.Acos(_cosTheta[jointNo]); // we get the radians

            // figure out what side are we on
            //if ((Vector3.Cross(tempLeftSide, tempRightSide).magnitude / (tempLeftSide.magnitude * tempRightSide.magnitude)) < 0) 
            //{
            //    degrees *= -1;
            //}

            degrees = (float)ClampPI(degrees) * Mathf.Rad2Deg;

            RotateJoint(jointNo, _rotationAxis[jointNo], degrees);

            if (Vector3.Distance(m_Joints[m_Joints.Count - 1].transform.position, m_TargetPoint.transform.position) <= m_DistanceFromTargetTolerance)
            {
                UnityEngine.Debug.Log(ChainName + " - iterations: " + iterationsXD + " | time elapsed: " + stopWatch.Elapsed + "  == TARGET REACHED ==");
                UnityEngine.Debug.Break();
                return;
            }
        }


    }

    void RotateJoint(int jointNo, Vector3 axis, float degrees)
    {
        m_Joints[jointNo].transform.Rotate(axis, degrees, Space.World);
    }

    double ClampPI(double theta)
    {
        theta = theta % (2.0 * Mathf.PI);
        if (theta < -Mathf.PI)
            theta += 2.0 * Mathf.PI;
        else if (theta > Mathf.PI)
            theta -= 2.0 * Mathf.PI;
        return theta;
    }
}
