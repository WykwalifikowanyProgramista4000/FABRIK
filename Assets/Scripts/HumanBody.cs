using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanBody : MonoBehaviour
{
    [Header("--- Components ---")]
    [SerializeField] private GameObject m_BodyBase;

    [Header("--- Targets ---")]
    [SerializeField] private GameObject m_LeftArmTarget;
    [SerializeField] private GameObject m_RightArmTarget;
    [SerializeField] private GameObject m_LeftLegTarget;
    [SerializeField] private GameObject m_RightLegTarget;

    [Header("--- Upper Chain ---")]
    [SerializeField] private GameObject m_upperChainSubbase;
    [SerializeField] private List<GameObject> m_upperChain = new List<GameObject>();

    [SerializeField] private List<GameObject> m_leftArm = new List<GameObject>();
    [SerializeField] private bool _leftArmLocked = false;
    [SerializeField] private bool _leftArmContinousFabrik = false;
    [SerializeField] private bool _leftArmConstraints = false;
    private FABRIKv3 _leftArmFabrik = new FABRIKv3();

    [SerializeField] private List<GameObject> m_RightArm = new List<GameObject>();
    [SerializeField] private bool _rightArmLocked = false;

    [SerializeField] private List<GameObject> m_Head = new List<GameObject>();
    [SerializeField] private bool _headLocked = false;

    [Header("--- Lower Chain ---")]
    [SerializeField] private GameObject m_lowerChainSubbase;
    [SerializeField] private List<GameObject> m_lowerChain = new List<GameObject>();    

    [SerializeField] private List<GameObject> m_leftLeg = new List<GameObject>();
    [SerializeField] private bool _leftLegLocked = true;

    [SerializeField] private List<GameObject> m_rightLeg = new List<GameObject>();
    [SerializeField] private bool _rightLegLocked = true;



    private void Start()
    {
        _leftArmFabrik.SetupFabrik(m_leftArm, m_upperChainSubbase, m_LeftArmTarget);
    }

    // Update is called once per frame
    void Update()
    {
        // Left Arm

        //if (_leftArmContinousFabrik) _leftArmFabrik.Fabrik();
        _leftArmFabrik._constraintsOn = _leftArmConstraints;

        _leftArmFabrik.FabrikForwards();

        // Right Arm



        // Left Leg



        // Right Leg



        // Head
    }
}
