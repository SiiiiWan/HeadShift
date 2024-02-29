using UnityEngine;
using System.Collections.Generic;
using UnityEngine.XR;
using System.Linq;
using System;

public class HeadShift : MonoBehaviour
{

    [Header("Transfer Function Settings")]

    [SerializeField] private float _GMax = 5.5f;
    [SerializeField] private float _Gmin = 0.5f;
    [SerializeField] private float _Inflection = 6.5f;
    [SerializeField] private float _K = 0.35f;
    [SerializeField] private float _speedThr = 0.2f;
    [SerializeField] private float _accThr = 1f;
    [SerializeField] private float _PMinUpward = 0.87f;
    [SerializeField] private float _FScale = 1.1f;
    [SerializeField] private float _boundingBoxRadius = 15;

    [SerializeField] private KeyCode ResetCursorPositionKey = KeyCode.Space;


    private Quaternion _camRotation
    {
        get { return Camera.main.transform.rotation; }
        set { }
    }  
    private Quaternion _preCamRotation; 

    private Vector3 _camPosition
    {
        get { return Camera.main.transform.position; }
        set { }
    }  
    private Vector3 _preCamPosition;

    private Ray _camRay
    {
        get { return new Ray(Camera.main.transform.position, Camera.main.transform.forward); }
        set { }
    }

    private Ray _cursorRay;
    public Ray CursorRay
    {
        get { return _cursorRay; }
        set { _cursorRay = value; }
    }

    private void Awake() 
    {          
        _headVelWindow = new List<float>();

        _headAccFilter = new OneEuroFilter(90f);
        _headVelFilter = new OneEuroFilter<Vector3>(90f);
    }

    private void Start()
    {
        _currentCursorRay = _camRay;
        _preCamPosition = _camPosition;
        _preCamRotation = _camRotation;

        deviceDetected = false;
        getHMD();
    }

    private void Update()
    {
        if(Input.GetKeyDown(ResetCursorPositionKey))
        {
            _currentCursorRay = _camRay;
        }

        if(!deviceDetected)
            getHMD();

        CursorRay = GetCursorRay();
    }

    private float _gain;
    private Ray _currentCursorRay;
    private Ray GetCursorRay()
    {
        _gain = UpdateGain();
        
        UpdateDeltaQuaternion();
        _currentCursorRay = AddOffset();

        _preCamPosition = _camPosition;
        _preCamRotation = _camRotation;

        return _currentCursorRay;
    }

    Quaternion _deltaQuaternion;
    void UpdateDeltaQuaternion()
    {
        _deltaQuaternion = _camRotation * Quaternion.Inverse(_preCamRotation);
    }

    Quaternion GetRotationOffset()
    {
        if(_gain < 1)
        {
            return Quaternion.Lerp(Quaternion.identity, _deltaQuaternion, _gain);
        }
        else
        {
            Quaternion rotation = _deltaQuaternion;

            for(int i = 0; i < Math.Floor(_gain) - 1; i++)
            {
                rotation = _deltaQuaternion * rotation;
            }

            if(Mathf.Approximately(_gain, Mathf.RoundToInt(_gain))) // int gain
                return rotation;

            return Quaternion.Lerp(rotation, _deltaQuaternion * rotation, _gain - (float)Math.Floor(_gain));
        }    
    }

    Ray AddOffset()
    {   
        Vector3 offsettedDir = GetRotationOffset() * _currentCursorRay.direction;

        _predictedHeadCursorAngle = Vector3.Angle(offsettedDir, Camera.main.transform.forward);

        bool _hitBound = (_predictedHeadCursorAngle > _boundingBoxRadius);

        Vector3 camDir = Camera.main.transform.forward;

        if(_hitBound)
        {
            Vector3 axis = Vector3.Cross(camDir, offsettedDir).normalized;       

            Quaternion rotation = Quaternion.AngleAxis(_boundingBoxRadius, axis);
            offsettedDir = rotation * camDir;                         
        } 

        return new Ray(_preCamPosition, offsettedDir);
    }


    private float _predictedHeadCursorAngle;
    private float _headCursorAngleVert;
    private float _vertCorrectionFactor;

    float UpdateGain()
    {
        UpdateHeadVel();
        UpdateHeadAcc();

        _vertCorrectionFactor = 1;

        float horiVelSmoothed = _filteredHeadVel.y;
        float vertVelSmoothed = _filteredHeadVel.x;

        if(_headCursorAngleVert > 0)
        {
            if (Mathf.Abs(_filteredHeadVel.magnitude) > _speedThr) // don't want change the gain pattern when refining
            {
                if(vertVelSmoothed > 0) _vertCorrectionFactor = (1 - (1-_PMinUpward) * Mathf.Sin(Mathf.Atan2(vertVelSmoothed, horiVelSmoothed)));
                else _vertCorrectionFactor = 1.1f;
            }                    
        }    


        if((Mathf.Abs(_filteredHeadVel.magnitude) < _speedThr) && (Mathf.Abs(_filteredHeadAcc) < _accThr))
        {
            return _vertCorrectionFactor * SigmoidFunction(_headVel, 1.2f, 0.5f, 0.3f, 7f);
        }

        return _FScale * _vertCorrectionFactor * SigmoidFunction(_headAcc, _GMax, _Gmin, _Inflection, _K);
    }

    float SigmoidFunction(float x, float max, float min, float middelX, float k)
    {
        return (1.0f / (1.0f + Mathf.Exp(k * (-x + middelX)))) * (max - min) + min;
    }

    private float _headVel;
    private Vector3 _filteredHeadVel;
    private List<float> _headVelWindow;
    private int _headVelWindowSize = 2;
    private OneEuroFilter<Vector3> _headVelFilter;

    void UpdateHeadVel()
    {   
        inputHeadset.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out Vector3 headAngularVel);

        _headVelWindow.Add(headAngularVel.magnitude);

        if(_headVelWindow.Count > _headVelWindowSize)
        {
            _headVelWindow.RemoveAt(0);
        }

        _filteredHeadVel = _headVelFilter.Filter(headAngularVel);
        _headVel = headAngularVel.magnitude;

    }

    private float _headAcc, _filteredHeadAcc;
    private OneEuroFilter _headAccFilter;

    void UpdateHeadAcc()
    {
        if(_headVelWindow.Count != _headVelWindowSize) return;

        float acc = (_headVelWindow[_headVelWindowSize - 1] - _headVelWindow[_headVelWindowSize - 2]) / Time.deltaTime;

        _filteredHeadAcc = _headAccFilter.Filter(acc);

        _headAcc = acc;
    }

    /// Detects HMD
    private List<InputDevice> devices;
    private InputDeviceCharacteristics desiredCharacteristics;
    private InputDevice inputHeadset;
    private bool deviceDetected;

    void getHMD(){

        devices = new List<InputDevice>();

        desiredCharacteristics = InputDeviceCharacteristics.HeadMounted;
        InputDevices.GetDevicesWithCharacteristics(desiredCharacteristics, devices);

        if(devices.Count > 0)
        {
            inputHeadset = devices[0];            
            deviceDetected = true;
        }
    }


}
