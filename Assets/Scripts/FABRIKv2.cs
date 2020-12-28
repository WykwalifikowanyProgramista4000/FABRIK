using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.UIElements;

public class FABRIKv2 : MonoBehaviour
{
    [Header("--- Components ---")]
    [SerializeField] private GameObject m_TargetPoint;
    [SerializeField] private GameObject m_Base;
    [SerializeField] private List<GameObject> m_Joints = new List<GameObject>();

    [Header("--- Settings ---")]
    [SerializeField] private int m_MaxFABRIKIterations = 20;
    [SerializeField] private float m_DistanceFromTargetTolerance = 0.01f;

    [Header("# DEBGUG #")]
    [SerializeField] private GameObject debug_TestPoint;
    [SerializeField] private float debug_RayDuration = 0.1f;

    private float[] _moduleLengths;
    List<Vector3> _joints;

    void Start()
    {
        _moduleLengths = new float[m_Joints.Count - 1];

        for (int i = 0; i < m_Joints.Count - 1; i++)
        {
            _moduleLengths[i] = Vector3.Distance(m_Joints[i].GetComponent<Transform>().position,
                                                 m_Joints[i + 1].GetComponent<Transform>().position);
        }

        List<Vector3> _joints = new List<Vector3>();
        foreach (GameObject jointObject in m_Joints)                            // filling the list of joints
        {
            _joints.Add(jointObject.GetComponent<Transform>().position);
        }
    }


    void Update()
    {
        Fabrik();
    }

    private void Fabrik()
    {
        int iterationCnt = 0;                                                   // seting the iteration constrain counter
        Vector3 target = m_TargetPoint.GetComponent<Transform>().position;      // saving the target global position
        Vector3 origin = m_Joints[0].GetComponent<Transform>().position;        // saving the origin global position

        if (Vector3.Distance(origin, target) > _moduleLengths.Sum(x => x))      // checking if target is within manipulators reach,
        {                                                                       //and if not it outstreaches thowards it to max lenght
            Vector3 normalDirectionVector = (target - origin).normalized;

            #region DEBUG: on
            Debug.DrawRay(origin, normalDirectionVector * 100, Color.red, debug_RayDuration, false);

            //get vector from point on line to point in space
            Vector3 linePointToPoint = debug_TestPoint.GetComponent<Transform>().position - _joints[0];
            float t = Vector3.Dot(linePointToPoint, normalDirectionVector);
            Vector3 O = _joints[0] + normalDirectionVector * t;
            O = O - origin;

            Debug.DrawRay(O + origin, (debug_TestPoint.GetComponent<Transform>().position - (O + origin)).normalized * 100, Color.magenta, debug_RayDuration, false);

            Vector3 XD = new Vector3(0,
                                     O.magnitude,
                                     0);

            Quaternion superCoolRotation = Quaternion.FromToRotation(XD, O);

            DrawStarAtPoint(XD + origin, Color.green);
            Debug.DrawRay(origin, XD.normalized * 100, Color.red, debug_RayDuration);

            Vector3 plusX = new Vector3(XD.x + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleX)),
                                XD.y,
                                XD.z);

            Vector3 minusX = new Vector3(XD.x - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().minAngleX)),
                                 XD.y,
                                 XD.z);

            Vector3 plusZ = new Vector3(XD.x,
                                XD.y,
                                XD.z + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleZ)));

            Vector3 minusZ = new Vector3(XD.x,
                                 XD.y,
                                 XD.z - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleZ)));

            Vector3 plusXplusZ = superCoolRotation * new Vector3(XD.x + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleX * Mathf.Deg2Rad)),
                                             XD.y,
                                             XD.z + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleZ * Mathf.Deg2Rad)));
            DrawStarAtPoint(plusXplusZ + origin, Color.red);

            Vector3 plusXminusZ = superCoolRotation * new Vector3(XD.x + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleX * Mathf.Deg2Rad)),
                                              XD.y,
                                              XD.z - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().minAngleZ * Mathf.Deg2Rad)));
            DrawStarAtPoint(plusXminusZ + origin, Color.red);

            Vector3 minusXplusZ = superCoolRotation * new Vector3(XD.x - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().minAngleX * Mathf.Deg2Rad)),
                                              XD.y,
                                              XD.z + (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().maxAngleZ * Mathf.Deg2Rad)));
            DrawStarAtPoint(minusXplusZ + origin, Color.red);

            Vector3 minusXminusZ = superCoolRotation * new Vector3(XD.x - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().minAngleX * Mathf.Deg2Rad)),
                                               XD.y,
                                               XD.z - (XD.y * Mathf.Tan(m_Joints[0].GetComponent<RobotJoint>().minAngleZ * Mathf.Deg2Rad)));
            DrawStarAtPoint(minusXminusZ + origin, Color.red);

            XD = superCoolRotation * XD;
            DrawStarAtPoint(XD + origin, Color.red);

            //Debug.DrawRay(plusXplusZ, (plusXminusZ - plusXplusZ).normalized * 100, Color.cyan, debug_RayDuration, false);
            //Debug.DrawRay(plusXplusZ, (minusXplusZ - plusXplusZ).normalized * 100, Color.cyan, debug_RayDuration, false);
            //Debug.DrawRay(minusXplusZ, (minusXminusZ - minusXplusZ).normalized * 100, Color.cyan, debug_RayDuration, false);
            //Debug.DrawRay(minusXminusZ, (plusXminusZ - minusXminusZ).normalized * 100, Color.cyan, debug_RayDuration, false);

            Debug.DrawLine(plusXplusZ + origin,
                           plusXminusZ + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + origin,
                           minusXplusZ + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + origin,
                           minusXminusZ + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + origin,
                           plusXminusZ + origin,
                           Color.cyan,
                           debug_RayDuration, false);

            Debug.DrawLine(plusXminusZ + origin,
                           XD + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + origin,
                           XD + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + origin,
                           XD + origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + origin,
                           XD + origin,
                           Color.cyan,
                           debug_RayDuration, false);

            Debug.DrawLine(plusXminusZ + origin,
                           origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + origin,
                            origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + origin,
                           origin,
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + origin,
                           origin,
                           Color.cyan,
                           debug_RayDuration, false);

            #endregion

            for (int i = 0; i < _joints.Count - 1; i++)
            {
                _joints[i + 1] = _joints[i] + normalDirectionVector * _moduleLengths[i];
            }
        }
        else
        {
            while (Vector3.Distance(_joints[_joints.Count - 1], target) > m_DistanceFromTargetTolerance &&  // if we are in reach we use FABRIK to determine the joints positions
                   iterationCnt < m_MaxFABRIKIterations)                                                // in global coordinates
            {
                FabrikBackward(_joints, target);
                FabrikForward(_joints, origin);
                iterationCnt++;
            }
        }

        //m_Joints[0].GetComponent<Transform>().rotation = Quaternion.LookRotation(joints[1] - origin) * Quaternion.AngleAxis(90, new Vector3(1,0,0));  // simply moving the joints to found positions - for debug

        RotateJoints(_joints);

    }


    private void FabrikBackward(List<Vector3> joints, Vector3 target)
    {
        joints[joints.Count - 1] = target;

        for (int i = joints.Count - 1; i > 0; i--)
        {
            joints[i - 1] = joints[i] + (joints[i - 1] - joints[i]).normalized * _moduleLengths[i - 1];
        }

    }


    private void FabrikForward(List<Vector3> joints, Vector3 origin)    // TODO: need to add constrains here! With them, I can locate new global position for the joint
    {
        Vector3 direction;
        Vector3 linePointToPoint;
        float t;
        GameObject O; ;

        Vector3 plusX;
        Vector3 minusX;
        Vector3 plusZ;
        Vector3 minusZ;

        GameObject plusXplusZ;
        GameObject plusXminusZ;
        GameObject minusXplusZ;
        GameObject minusXminusZ;

        // place joint[0] on the origin with its Z axis pointing along the direction vector cast from the base to joint[0]
        joints[0] = origin;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            if (i > 0)
            {
                #region DEBUG: on : casting the line along the arm
                direction = (joints[i] - joints[i - 1]).normalized;

                // Linia wzdłóż lokalnej osi Y
                Debug.DrawRay(joints[i], direction.normalized * 100, Color.red, debug_RayDuration, false);
                //Debug.DrawRay(joints[i], direction.normalized * -100, Color.red, debug_RayDuration, false);
                #endregion
            }
            else
            {
                #region DEBUG: on : casting the line along the arm
                direction = (joints[0] - m_Base.GetComponent<Transform>().position).normalized;

                // Linia wzdłóż lokalnej osi Y
                Debug.DrawRay(joints[0], direction.normalized * 100, Color.red, debug_RayDuration, false);
                //Debug.DrawRay(joints[0], direction.normalized * -100, Color.red, debug_RayDuration, false);
                #endregion
            }

            #region 1. Find new unconstraind position for i+1 joint
            joints[i + 1] = joints[i] + (joints[i + 1] - joints[i]).normalized * _moduleLengths[i];
            #endregion

            #region 2. Cast a line L from i joint that goes through i and i-1 joint and find the point O on the newly created line at the intersection with the perpendicular line cast to L
            //O = Instantiate<GameObject>(m_Joints[i + 1], m_Joints[i+1].GetComponent<Transform>());
            ////get vector from point on line to point in space
            //linePointToPoint = joints[i + 1] - joints[i];
            //t = Vector3.Dot(linePointToPoint, direction);
            //Vector3 oPosition = new Vector3() + joints[i] + direction * t;
            //O.GetComponent<Transform>().position = joints[i] + direction * t;
            //Debug.DrawRay(O.GetComponent<Transform>().position, (joints[i + 1] - O.GetComponent<Transform>().position).normalized * 100, Color.magenta, debug_RayDuration, false);
            #endregion

            #region 3. Find the edges of the constraint that are on the same plane as the unconstrained joint and point O


            //plusXplusZ = new Vector3(O.x + (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleX)),
            //                         O.y,
            //                         O.z + (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleZ)));

            //plusXminusZ = new Vector3(O.x + (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleX)),
            //                         O.y,
            //                         O.z - (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleZ)));

            //minusXplusZ = new Vector3(O.x - (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().minAngleX)),
            //                          O.y,
            //                          O.z + (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleZ)));

            //minusXminusZ = new Vector3(O.x - (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().minAngleX)),
            //                           O.y,
            //                           O.z - (O.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleZ)));
            #endregion
        }
    }

    // After constrained positions are decided, I phisically rotate the joints to express the way in which kinematich chain with all its parts looks like 
    // It seems like the first straight forward aproach will now be ok enough for that task, as it is simple, fast and easy on resources :)
    private void RotateJoints(List<Vector3> joints)
    {
        //Quaternion angles;
        for (int i = 0; i < joints.Count - 1; i++)  // obtaining the needed rotation for the fount joint positions
        {
            m_Joints[i].GetComponent<Transform>().rotation = Quaternion.LookRotation((joints[i + 1] - m_Joints[i].GetComponent<Transform>().position).normalized) * Quaternion.AngleAxis(90, new Vector3(1, 0, 0));
        }

        #region DEBUG: off
        ////  0 -> 1
        //Vector3 direction = (joints[1] - joints[0]).normalized;

        //Ray ray = new Ray(joints[0], direction);    // Linia wzdłóż lokalnej osi Y
        //Debug.DrawRay(joints[0], direction.normalized * 100, Color.red, 0.1f, false);
        //Debug.DrawRay(joints[0], direction.normalized * -100, Color.red, 0.1f, false);

        ////get vector from point on line to point in space
        //Vector3 linePointToPoint = m_debug_TestPoint.GetComponent<Transform>().position - joints[0];
        //float t = Vector3.Dot(linePointToPoint, direction);
        //Vector3 O = joints[0] + direction * t;

        //Debug.DrawRay(O, (m_debug_TestPoint.GetComponent<Transform>().position - O).normalized * 100, Color.magenta, 0.1f, false);

        ////  1 -> 2
        //direction = (joints[2] - joints[1]).normalized;

        //ray = new Ray(joints[1], direction);    // Linia wzdłóż lokalnej osi Y
        //Debug.DrawRay(joints[1], direction.normalized * 100, Color.red, 0.1f, false);
        //Debug.DrawRay(joints[1], direction.normalized * -100, Color.red, 0.1f, false);

        ////get vector from point on line to point in space
        //linePointToPoint = m_debug_TestPoint.GetComponent<Transform>().position - joints[1];
        //t = Vector3.Dot(linePointToPoint, direction);
        //O = joints[1] + direction * t;

        //Debug.DrawRay(O, (m_debug_TestPoint.GetComponent<Transform>().position - O).normalized * 100, Color.cyan, 0.1f, false);
        #endregion
    }

    private void DrawStarAtPoint(Vector3 point, Color color)
    {
        float armLength = 0.05f;

        Debug.DrawRay(point, (new Vector3(0, 0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(0, 1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 1, 0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 0, 0)).normalized * armLength, color, debug_RayDuration, false);

        Debug.DrawRay(point, (new Vector3(-0, -0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-0, -1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -1, -0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -0, -0)).normalized * armLength, color, debug_RayDuration, false);

        Debug.DrawRay(point, (new Vector3(-0, -0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-0, -1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -1, 0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, -0, 0)).normalized * armLength, color, debug_RayDuration, false);

        Debug.DrawRay(point, (new Vector3(-0, 0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-0, 1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 1, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 1, 0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 0, 1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 0, 0)).normalized * armLength, color, debug_RayDuration, false);

        Debug.DrawRay(point, (new Vector3(-0, 0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-0, 1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 1, -0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(-1, 0, -0)).normalized * armLength, color, debug_RayDuration, false);

        Debug.DrawRay(point, (new Vector3(0, 0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(0, 1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 1, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 1, -0)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 0, -1)).normalized * armLength, color, debug_RayDuration, false);
        Debug.DrawRay(point, (new Vector3(1, 0, -0)).normalized * armLength, color, debug_RayDuration, false);
    }
}

