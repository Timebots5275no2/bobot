using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CamFollow : MonoBehaviour
{
    [SerializeField] Transform objectToFollow;

    Quaternion initRot;
    Input input;
    Vector3 offset;

    bool isChilded = false;

    void Awake()
    {
        initRot = transform.rotation;
        offset = transform.position - objectToFollow.transform.position;

        input = new Input();
        input.Yourmother.Enable();
        input.Yourmother.SwitchCameraMode.performed += SwitchCameraMode;
    }

    // Update is called once per frame
    void Update()
    {
        if (!isChilded)
        {
            transform.position = objectToFollow.position + offset;
        }
    }

    void SwitchCameraMode(InputAction.CallbackContext context)
    {
        isChilded = !isChilded;

        if (isChilded) { transform.SetParent(objectToFollow); transform.localPosition = offset; }
        else { transform.SetParent(null); }
        transform.localRotation = initRot;
    }
}
