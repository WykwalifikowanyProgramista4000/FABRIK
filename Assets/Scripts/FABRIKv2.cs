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
    }

    private void Fabrik()
    {
        int iterationCnt = 0;                                                   // seting the iteration constrain counter
        Vector3 target = m_TargetPoint.GetComponent<Transform>().position;      // saving the target global position
        Vector3 origin = m_Joints[0].GetComponent<Transform>().position;        // saving the origin global position

        List<Vector3> joints = new List<Vector3>();                             // making the list od joints
        foreach (GameObject jointObject in m_Joints)                            // filling the list of joints
        {
            joints.Add(jointObject.GetComponent<Transform>().position);
        }

        //if (Vector3.Distance(origin, target) > _moduleLengths.Sum(x => x))      // checking if target is within manipulators reach,
        if (false)
        {                                                                       //and if not it outstreaches thowards it to max lenght
            Vector3 normalDirectionVector = (target - origin).normalized;

            for (int i = 0; i < joints.Count - 1; i++)
            {
                joints[i + 1] = joints[i] + normalDirectionVector * _moduleLengths[i];
            }
        }
        else
        {
            do
            {
                FabrikBackward(joints, target);
                FabrikForward(joints, origin);
                iterationCnt++;
            } while (Vector3.Distance(joints[joints.Count - 1], target) > m_DistanceFromTargetTolerance &&  // if we are in reach we use FABRIK to determine the joints positions
                   iterationCnt < m_MaxFABRIKIterations);                                                // in global coordinates
        }

        //m_Joints[0].GetComponent<Transform>().rotation = Quaternion.LookRotation(joints[1] - origin) * Quaternion.AngleAxis(90, new Vector3(1,0,0));  // simply moving the joints to found positions - for debug

        RotateJoints(joints);
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
        Vector3 O; //we need to store information about rotation of this object, we probably could use Transform component, but want to be flexible for now :V

        Vector3 plusX;
        Vector3 minusX;
        Vector3 plusZ;
        Vector3 minusZ;

        Vector3 plusXplusZ;
        Vector3 plusXminusZ;
        Vector3 minusXplusZ;
        Vector3 minusXminusZ;

        bool isOBelowJoint = false;

        // place joint[0] on the origin with its Z axis pointing along the direction vector cast from the base to joint[0]
        joints[0] = origin;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            if (i > 0)
            {
                #region DEBUG: off : casting the line along the arm
                direction = (joints[i] - joints[i - 1]).normalized;

                // Linia wzdłóż lokalnej osi Y
                //Debug.DrawRay(joints[i], direction.normalized * 100, Color.red, debug_RayDuration, false);
                //Debug.DrawRay(joints[i], direction.normalized * -100, Color.red, debug_RayDuration, false);
                #endregion
            }
            else
            {
                #region DEBUG: off : casting the line along the arm
                direction = (joints[0] - m_Base.GetComponent<Transform>().position).normalized;

                // Linia wzdłóż lokalnej osi Y
                //Debug.DrawRay(joints[0], direction.normalized * 100, Color.red, debug_RayDuration, false);
                //Debug.DrawRay(joints[0], direction.normalized * -100, Color.red, debug_RayDuration, false);
                #endregion
            }

            #region 1. Find new unconstraind position for i+1 joint
            joints[i + 1] = joints[i] + (joints[i + 1] - joints[i]).normalized * _moduleLengths[i];
            #endregion

            #region 2. Cast a line L from i joint that goes through i and i-1 joint and find the point O on the newly created line at the intersection with the perpendicular line cast to L
            linePointToPoint = (joints[i + 1] - joints[i]); //get vector from point on line to point in space
            t = Vector3.Dot(linePointToPoint, direction);
            O = joints[i] + direction * t;
            #endregion

            #region 3. Find the edges of the constraint that are on the same plane as the unconstrained joint and point O

            O = O - joints[i];

            Vector3 XD = new Vector3(0,
                                     O.magnitude,
                                     0);

            Quaternion vecB2vecAQuaternion = Quaternion.FromToRotation(XD.normalized, O.normalized);
            Quaternion modelBaseQuaternion = Quaternion.AngleAxis(m_Base.GetComponent<Transform>().rotation.eulerAngles.y, Vector3.up);

            Quaternion temp = vecB2vecAQuaternion * modelBaseQuaternion;

            float yAxisAngleZ = XD.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleZ * Mathf.Deg2Rad);
            float yAxisAngleX = XD.y * Mathf.Tan(m_Joints[i].GetComponent<RobotJoint>().maxAngleX * Mathf.Deg2Rad);

            plusXplusZ = temp *
                                new Vector3(XD.x + yAxisAngleX,
                                XD.y,
                                XD.z + yAxisAngleZ);


            plusXminusZ = temp *
                                new Vector3(XD.x + yAxisAngleX,
                                XD.y,
                                XD.z - yAxisAngleZ);


            minusXplusZ = temp *
                                new Vector3(XD.x - yAxisAngleX,
                                XD.y,
                                XD.z + yAxisAngleZ);


            minusXminusZ = temp *
                                 new Vector3(XD.x - yAxisAngleX,
                                 XD.y,
                                 XD.z - yAxisAngleZ);

            XD = vecB2vecAQuaternion * XD;
            #endregion

            #region 4. Check if the position of point O is above or below joint[i] on ray formed from joint[i-1] through joint[i]. If below, flip the constraint volume

            //as point O must be on the said line, we can simply check if the distance O <-> joint[i] is greater than distance O <-> joint[i-1]
            //if(i>0 && (O + joints[i] - joints[i - 1]).magnitude < (joints[i] - joints[i-1]).magnitude)
            if (i > 0 && Vector3.Angle(joints[i + 1] - joints[i], joints[i - 1] - joints[i]) < 90)
            {
                isOBelowJoint = true;
            }

            #endregion

            #region 5. Get crossing point of constraint volume edge and unconstrained joint[i+1] position

            float a=0;
            float alpha;
            float beta;
            float gamma;
            float theta = Vector3.Angle(plusXplusZ-O, plusXminusZ-O);

            Vector3 OXD = joints[i + 1] - O - joints[i];
            Vector3 constrainedJointNextPosition = new Vector3();
            
            if (i == joints.Count - 2)
            {
                Debug.Log("plusXplusZ: " + Vector3.Angle(OXD, plusXplusZ - O) + "   minusXminusZ: " + Vector3.Angle(OXD, minusXminusZ - O) +
                    "   plusXminusZ: " + Vector3.Angle(OXD, plusXminusZ - O) + "   minusXplusZ: " + Vector3.Angle(OXD, minusXplusZ - O) +
                    "   angle: " + Vector3.Angle(OXD, (plusXplusZ - plusXminusZ) / 2) + "   theta: " + theta +
                    "   isOBelowJoint: " + isOBelowJoint);
            }

            if (Vector3.Angle(OXD, (plusXplusZ + plusXminusZ) / 2 - O) < theta / 2) // ćwiartka 1
            {
                alpha = Vector3.Angle((joints[i+1] - (O + joints[i])), plusXminusZ - O);
                beta = Vector3.Angle((plusXplusZ - plusXminusZ), (O - plusXminusZ));
                gamma = 180 - alpha - beta;
                a = Vector3.Magnitude(plusXminusZ - O) * Mathf.Sin(alpha * Mathf.Deg2Rad) / Mathf.Sin(gamma * Mathf.Deg2Rad);

                constrainedJointNextPosition = (plusXplusZ - plusXminusZ).normalized * a + plusXminusZ + joints[i];

                Debug.DrawRay(O + joints[i], ((plusXplusZ + plusXminusZ) / 2 - O).normalized * 100, Color.yellow, debug_RayDuration);
                DrawStarAtPoint(constrainedJointNextPosition, Color.yellow);
                DrawStarAtPoint(plusXminusZ + joints[i], Color.yellow);
            }
            else if (Vector3.Angle(OXD, (plusXplusZ + plusXminusZ) / 2 - O) > 180 - (theta / 2)) // ćwiartka 3
            {
                alpha = Vector3.Angle((joints[i + 1] - (O + joints[i])), minusXplusZ - O);
                beta = Vector3.Angle((minusXminusZ - minusXplusZ), (O - minusXplusZ));
                gamma = 180 - alpha - beta;
                a = Vector3.Magnitude(minusXplusZ - O) * Mathf.Sin(alpha * Mathf.Deg2Rad) / Mathf.Sin(gamma * Mathf.Deg2Rad);

                constrainedJointNextPosition = (minusXminusZ - minusXplusZ).normalized * a + minusXplusZ + joints[i];

                Debug.DrawRay(O + joints[i], ((plusXplusZ + plusXminusZ) / 2 - O).normalized * -100, Color.green, debug_RayDuration);
                DrawStarAtPoint(constrainedJointNextPosition, Color.green);
                DrawStarAtPoint(minusXplusZ + joints[i], Color.green);
            }
            else if (Vector3.Angle(OXD, (plusXplusZ + minusXplusZ) / 2 - O) < (180 - theta) / 2) // ćwiartka 4
            {
                alpha = Vector3.Angle((joints[i + 1] - (O + joints[i])), plusXplusZ - O);
                beta = Vector3.Angle((minusXplusZ - plusXplusZ), (O - plusXplusZ));
                gamma = 180 - alpha - beta;
                a = Vector3.Magnitude(plusXplusZ - O) * Mathf.Sin(alpha * Mathf.Deg2Rad) / Mathf.Sin(gamma * Mathf.Deg2Rad);

                constrainedJointNextPosition = (minusXplusZ - plusXplusZ).normalized * a + plusXplusZ + joints[i];

                Debug.DrawRay(O + joints[i], ((plusXplusZ + minusXplusZ) / 2 - O).normalized * 100, Color.red, debug_RayDuration);
                DrawStarAtPoint(constrainedJointNextPosition, Color.red);
                DrawStarAtPoint(plusXplusZ + joints[i], Color.red);
            }
            else // ćwiartka 2
            {
                alpha = Vector3.Angle((joints[i + 1] - (O + joints[i])), minusXminusZ - O);
                beta = Vector3.Angle((plusXminusZ - minusXminusZ), (O - minusXminusZ));
                gamma = 180 - alpha - beta;
                a = Vector3.Magnitude(minusXminusZ - O) * Mathf.Sin(alpha * Mathf.Deg2Rad) / Mathf.Sin(gamma * Mathf.Deg2Rad);

                constrainedJointNextPosition = (plusXminusZ - minusXminusZ).normalized * a + minusXminusZ + joints[i];

                Debug.DrawRay(O + joints[i], ((minusXminusZ + plusXminusZ) / 2 - O).normalized * 100, Color.magenta, debug_RayDuration);
                DrawStarAtPoint(constrainedJointNextPosition, Color.magenta);
                DrawStarAtPoint(minusXminusZ + joints[i], Color.magenta);
            }

            #endregion

            #region 6. Correct next joint position according to constrains
            
            if (isOBelowJoint)
            {
                Vector3 T = (constrainedJointNextPosition - joints[i]).normalized * _moduleLengths[i];
                float L = Vector3.Dot(T, (joints[i - 1] - joints[i]).normalized);

                Vector3 M = (joints[i - 1] - joints[i]).normalized * L;
                Vector3 Mprime = (joints[i] - joints[i-1]).normalized * L;
                Vector3 Tprime = T - M + Mprime;

                DrawStarAtPoint(joints[i], Color.red);
                DrawStarAtPoint(T + joints[i], Color.red);
                DrawStarAtPoint(M + joints[i], Color.green);

                Debug.DrawLine(joints[i],
                          M + joints[i],
                          Color.green,
                          debug_RayDuration, false);

                DrawStarAtPoint(Mprime + joints[i], Color.yellow);
                DrawStarAtPoint(Tprime + joints[i], Color.magenta);

                joints[i + 1] = joints[i] + Tprime;
            }
            else if (Vector3.Distance(joints[i+1], O + joints[i]) > Vector3.Distance(constrainedJointNextPosition, O + joints[i]))
            {
                joints[i + 1] = (constrainedJointNextPosition - joints[i]).normalized * _moduleLengths[i] + joints[i];
            }

            #endregion

            #region 7. End statements
            
            isOBelowJoint = false;
            
            #endregion

            #region Debug: on
            Debug.DrawRay(O+joints[i], (joints[i + 1] - (O + joints[i])).normalized * 100, Color.magenta, debug_RayDuration, false);

            //DrawStarAtPoint(XD + joints[i], Color.green);
            //DrawStarAtPoint(O + joints[i], Color.blue);
            //DrawStarAtPoint(plusXplusZ + joints[i], Color.red);
            //DrawStarAtPoint(plusXminusZ + joints[i], Color.magenta);
            //DrawStarAtPoint((joints[i+1] - (XD + joints[i])).normalized * (float)a + XD + joints[i], Color.yellow);
            //DrawStarAtPoint((plusXplusZ - plusXminusZ).normalized * a + plusXminusZ + joints[i], Color.yellow);
            //DrawStarAtPoint((joints[i + 1] - O - joints[i]).normalized * b + O + joints[i], Color.yellow);

            Debug.DrawLine(plusXplusZ + joints[i],
                           plusXminusZ + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + joints[i],
                           minusXplusZ + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + joints[i],
                           minusXminusZ + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + joints[i],
                           plusXminusZ + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);

            Debug.DrawLine(plusXminusZ + joints[i],
                           O + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + joints[i],
                           O + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + joints[i],
                           O + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + joints[i],
                           O + joints[i],
                           Color.cyan,
                           debug_RayDuration, false);

            Debug.DrawLine(plusXminusZ + joints[i],
                           joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(plusXplusZ + joints[i],
                           joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXplusZ + joints[i],
                           joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            Debug.DrawLine(minusXminusZ + joints[i],
                           joints[i],
                           Color.cyan,
                           debug_RayDuration, false);
            #endregion
        }
    }

    // After constrained positions are decided, I phisically rotate the joints to express the way in which kinematich chain with all its parts looks like 
    // It seems like the first straight forward aproach will now be ok enough for that task, as it is simple, fast and easy on resources :)
    private void RotateJoints(List<Vector3> joints)
    {
        //Quaternion angles;
        for (int i = 0; i < joints.Count - 1; i++)  // obtaining the needed rotationWebexowi ns for the fount joint positions
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
