using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.UIElements;

public class CCD : MonoBehaviour
{
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

    private void Start()
    {
        _cosTheta = new float[m_Joints.Count - 1];
        _rotationAxis = new Vector3[m_Joints.Count];
    }

    private void Update()
    {
        SolveCCD();
    }

    private void SolveCCD()
    {
        Vector3 tempLeftSide;
        Vector3 tempRightSide;

        float tempLeftNormal;
        float tempRightNormal;

        //max error
        if(Vector3.Distance(m_Joints[m_Joints.Count-1].transform.position, m_TargetPoint.transform.position) <= m_DistanceFromTargetTolerance)
        {
            return;
        }

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

            degrees = (float)RemovePiPeriod(degrees) * Mathf.Rad2Deg;

            RotateJoint(jointNo, _rotationAxis[jointNo], degrees);
        }


    }

    void RotateJoint(int jointNo, Vector3 axis, float degrees)
    {
        m_Joints[jointNo].transform.Rotate(axis, degrees, Space.World);
    }

    double RemovePiPeriod(double theta)
    {
        theta = theta % (2.0 * Mathf.PI);
        if (theta < -Mathf.PI)
            theta += 2.0 * Mathf.PI;
        else if (theta > Mathf.PI)
            theta -= 2.0 * Mathf.PI;
        return theta;
    }
}
