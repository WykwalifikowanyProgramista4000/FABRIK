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
    [SerializeField] private bool _upperChainLocked = false;
    [SerializeField] private bool _upperChainContinousFabrik = false;
    [SerializeField] private bool _upperChainConstraints = false;
    private FABRIKv3 _upperChainFabrik = new FABRIKv3();

    [SerializeField] private List<GameObject> m_leftArm = new List<GameObject>();
    [SerializeField] private bool _leftArmLocked = false;
    [SerializeField] private bool _leftArmContinousFabrik = false;
    [SerializeField] private bool _leftArmConstraints = false;
    private FABRIKv3 _leftArmFabrik = new FABRIKv3();

    [SerializeField] private List<GameObject> m_rightArm = new List<GameObject>();
    [SerializeField] private bool _rightArmLocked = false;
    [SerializeField] private bool _rightArmContinousFabrik = false;
    [SerializeField] private bool _rightArmConstraints = false;
    private FABRIKv3 _rightArmFabrik = new FABRIKv3();

    [SerializeField] private List<GameObject> m_Head = new List<GameObject>();
    [SerializeField] private bool _headLocked = false;

    [Header("--- Lower Chain ---")]
    [SerializeField] private GameObject m_lowerChainSubbase;
    [SerializeField] private List<GameObject> m_lowerChain = new List<GameObject>();    

    [SerializeField] private List<GameObject> m_leftLeg = new List<GameObject>();
    [SerializeField] private bool _leftLegLocked = true;

    [SerializeField] private List<GameObject> m_rightLeg = new List<GameObject>();
    [SerializeField] private bool _rightLegLocked = true;

    [Header("~~~~ DEBUG ~~~~")]
    [SerializeField] private bool _doFabrikForwardsOnce = false;
    [SerializeField] private bool _doFabrikForwardsContinously = false;

    [SerializeField] private bool _doFabrikBackwardsOnce = false;
    [SerializeField] private bool _doFabrikBackwardsContinously = false;

    [SerializeField] private bool _LeftArmDoFabrikBackwardsOnce = false;
    [SerializeField] private bool _LeftArmDoFabrikForwardsOnce = false;



    private void Start()
    {
        _leftArmFabrik.SetupFabrik(m_leftArm, m_BodyBase, m_LeftArmTarget);
        //_leftArmFabrik.SetupFabrik(m_leftArm, m_upperChain[m_upperChain.Count - 1], m_LeftArmTarget);
        _rightArmFabrik.SetupFabrik(m_rightArm, m_upperChain[m_upperChain.Count - 1], m_RightArmTarget);
        _upperChainFabrik.SetupFabrik(m_upperChain, m_BodyBase, m_upperChainSubbase);
    }

    // Update is called once per frame
    void Update()
    {
        _leftArmFabrik._constraintsOn = _leftArmConstraints;
        _rightArmFabrik._constraintsOn = _rightArmConstraints;
        _upperChainFabrik._constraintsOn = _upperChainConstraints;


        if (_doFabrikForwardsOnce || _doFabrikForwardsContinously)
        {
            _leftArmFabrik.Forwards();
            _rightArmFabrik.Forwards();

            DetermineSubbasePosition(m_upperChainSubbase, new List<GameObject>() { m_leftArm[0], m_rightArm[0] });

            _upperChainFabrik.Forwards();

            _doFabrikForwardsOnce = false;
        }

        if (_doFabrikBackwardsOnce || _doFabrikBackwardsContinously)
        {
            _upperChainFabrik.Backwards();

            //m_upperChainSubbase.transform.position = m_upperChain[m_upperChain.Count - 1].transform.position;

            //_leftArmFabrik.Backwards();
            //_rightArmFabrik.Backwards();


            _doFabrikBackwardsOnce = false;
        }

        if (_LeftArmDoFabrikBackwardsOnce)
        {
            _leftArmFabrik.Backwards();
            _LeftArmDoFabrikBackwardsOnce = false;
        }
    }

    void DetermineSubbasePosition(GameObject subbase , List<GameObject> chainBases)
    {
        float magnitudes = 0;
        Vector3 normals = new Vector3(0, 0, 0);

        foreach (GameObject chainBase in chainBases)
        {
            magnitudes += chainBase.transform.position.magnitude;
            normals += chainBase.transform.position.normalized;
        }

        magnitudes = magnitudes / chainBases.Count;
        normals = normals / chainBases.Count;

        subbase.transform.position = normals * magnitudes;
    }
}
