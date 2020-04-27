using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct AngleRange
{
    [SerializeField] public float minAngle;
    [SerializeField] public float maxAngle;

    public AngleRange(float min = 0, float max = 0)
    {
        minAngle = min;
        maxAngle = max;
    }
}

public class RobotJoint : MonoBehaviour
{
    [Header("--- Axis Freedom ---")]
    public Vector3 Axis;

    [Header("- X -")]
    public float minAngleX;
    public float maxAngleX;
    [Header("- Y -")]
    public float minAngleY;
    public float maxAngleY;
    [Header("- Z -")]
    public float minAngleZ;
    public float maxAngleZ;


    [Header("~ Not Implemented ~")]
    public Vector3 StartOffset;
    public float minAngle;
    public float maxAngle;

    void Start()
    {
        StartOffset = transform.localPosition;
    }
}
