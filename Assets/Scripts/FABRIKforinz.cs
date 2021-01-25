using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.UIElements;

public class FABRIKforinz : MonoBehaviour
{
    public string ChainName = "unnamed fabrik";

    [Header("--- Components ---")]
    [SerializeField] private GameObject m_TargetPoint;
    [SerializeField] private List<GameObject> m_Joints = new List<GameObject>();

    [Header("--- Settings ---")]
    [SerializeField] private float m_DistanceFromTargetTolerance = 0.01f;

    private float[] _moduleLengths;

    private int iterationsXD = 0;
    Stopwatch stopWatch = new Stopwatch();


    void TEST()
    {
        iterationsXD++;

        if (iterationsXD == 1 || iterationsXD == 33 || iterationsXD == 12)
        {
            float distance = Vector3.Distance(m_Joints[m_Joints.Count - 1].transform.position, m_TargetPoint.transform.position);
            UnityEngine.Debug.Log(ChainName + " - iterations: " + iterationsXD + " | distance: " + distance + " | time elapsed: " + stopWatch.Elapsed);

            UnityEngine.Debug.Break();
        }
    }

    void Start()
    {
        _moduleLengths = new float[m_Joints.Count - 1];

        for (int i = 0; i < m_Joints.Count - 1; i++)
        {
            _moduleLengths[i] = Vector3.Distance(m_Joints[i].GetComponent<Transform>().position,
                                                 m_Joints[i + 1].GetComponent<Transform>().position);
        }
    }


    void Update()
    {
        Fabrik();
        TEST();
    }

    private void Fabrik()
    {
        if (Vector3.Distance(m_Joints[m_Joints.Count - 1].transform.position, m_TargetPoint.transform.position) <= m_DistanceFromTargetTolerance)
        {
            UnityEngine.Debug.Log(ChainName + " - iterations: " + iterationsXD + " | time elapsed: " + stopWatch.Elapsed + " | TARGET REACHED");
            UnityEngine.Debug.Break();
            return;
        }
        
        Vector3 target = m_TargetPoint.GetComponent<Transform>().position;      // saving the target global position
        Vector3 origin = m_Joints[0].GetComponent<Transform>().position;        // saving the origin global position

        List<Vector3> joints = new List<Vector3>();                             // making the list od joints
        foreach (GameObject jointObject in m_Joints)                            // filling the list of joints
        {
            joints.Add(jointObject.GetComponent<Transform>().position);
        }

        stopWatch.Start();
        ForwardReach(joints, target);
        BackwardReach(joints, origin);
        stopWatch.Stop();


        RotateJoints(joints);
    }


    private void ForwardReach(List<Vector3> joints, Vector3 target)
    {
        joints[joints.Count - 1] = target;

        for (int i = joints.Count - 1; i > 0; i--)
        {
            joints[i - 1] = joints[i] + (joints[i - 1] - joints[i]).normalized * _moduleLengths[i - 1];
        }

    }

    private void BackwardReach(List<Vector3> joints, Vector3 origin)
    {
        joints[0] = origin;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            joints[i + 1] = joints[i] + (joints[i + 1] - joints[i]).normalized * _moduleLengths[i];
        }
    }

    private void RotateJoints(List<Vector3> joints)
    {
        for (int i = 0; i < joints.Count - 1; i++)
        {
            m_Joints[i].GetComponent<Transform>().rotation = Quaternion.LookRotation((joints[i + 1] - m_Joints[i].GetComponent<Transform>().position).normalized) * Quaternion.AngleAxis(90, new Vector3(1, 0, 0));
        }
    }
}