using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIR_IK_RR : MonoBehaviour
{
    [SerializeField] private GameObject m_TargetPoint;
    [SerializeField] private GameObject m_Effector;
    [SerializeField] private List<GameObject> Joints = new List<GameObject>(2);

   // private Vector3 _targetPoint;
    //private Transform _effectorTransform;

    private float _lengthA;
    private float _lengthB;

    void Start()
    {
        _lengthA = Vector3.Distance(Joints[0].GetComponent<Transform>().position,
                                    Joints[1].GetComponent<Transform>().position);

        _lengthB = Vector3.Distance(Joints[1].GetComponent<Transform>().position,
                                    m_Effector.GetComponent<Transform>().position);

        Debug.Log("_lengthA: " + _lengthA + "\n_lengthB: " + _lengthB);
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            InverseKinematics();
        }
    }

    private void InverseKinematics()
    {
        Vector3 targetPoint = m_TargetPoint.GetComponent<Transform>().position;

        double alpha1;
        double alpha2;

        double L;    // distance betwen origin point (Joint 0) and the target point (effector)

        Debug.Log("z: " + Joints[0].GetComponent<Transform>().position.z + "   y: " + Joints[0].GetComponent<Transform>().position.z);

        L = (Math.Pow(Joints[0].GetComponent<Transform>().position.z - targetPoint.z, 2) + Math.Pow(Joints[0].GetComponent<Transform>().position.y - targetPoint.y, 2) - Math.Pow(_lengthA, 2) - Math.Pow(_lengthB, 2)) / (2 * _lengthA * _lengthB);

        alpha2 = Math.Atan(Math.Sqrt(1 - Math.Pow(L, 2)) / L);

        alpha1 = (Math.Atan(Math.Abs(Joints[0].GetComponent<Transform>().position.y - targetPoint.y / Joints[0].GetComponent<Transform>().position.z - targetPoint.z)) - Math.Atan(_lengthB * Math.Sin(alpha2) / (_lengthA + _lengthB * Math.Cos(alpha2))));

        Joints[0].GetComponent<Transform>().rotation = Quaternion.AngleAxis((float)alpha1 * Mathf.Rad2Deg, Joints[0].GetComponent<RobotJoint>().Axis);
        Joints[1].GetComponent<Transform>().rotation = Quaternion.AngleAxis((float)(alpha1+ alpha2) * Mathf.Rad2Deg, Joints[1].GetComponent<RobotJoint>().Axis);

        Debug.Log("alpha1: " + alpha1 * Mathf.Rad2Deg + "\nalpha2: " + alpha2 * Mathf.Rad2Deg);
        
    }


}
