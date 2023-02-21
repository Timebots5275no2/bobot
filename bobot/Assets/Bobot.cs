using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Bobot : MonoBehaviour
{
    [SerializeField] float moveSpeed;
    [SerializeField] float rotateSpeed;

    Input input;
    Vector2 movementVal;
    float rotateVal;

    float rotation;
    Vector3 normalVector;

    float zero;

    private void Start()
    {
        input = new Input();
        input.Yourmother.Enable();
        input.Yourmother.ResetZero.performed += NewZero;

        zero = rotation;
    }

    private void Update()
    {
        sus();
        GetInput();
        Rotate();
        Drive(movementVal, rotation);

        Debug.DrawLine(transform.position, transform.position + transform.forward * 3, Color.green);
    }

    void GetInput()
    {
        movementVal = input.Yourmother.Movement.ReadValue<Vector2>().normalized;
        rotateVal = input.Yourmother.Rotation.ReadValue<float>();

        rotation += rotateVal * Time.deltaTime * rotateSpeed;
        rotation += rotation < 0 ? 360 : 0;
        rotation -= rotation > 360 ? 360 : 0;
    }

    void Rotate()
    {
        Vector2 sussyMcBalls = RadToVector2(rotation * Mathf.Deg2Rad);
        transform.rotation = Quaternion.LookRotation(new Vector3(sussyMcBalls.x, 0, sussyMcBalls.y), normalVector); 
    }

    void Drive(Vector2 direction, float rot)
    {
        Vector2 rot2D = new Vector2(transform.forward.x, transform.forward.z);
        Vector2 rotated = RotateVectorByDegrees(direction, (Vector2ToRad(rot2D) * Mathf.Rad2Deg) + rot - zero - 90);
        transform.position += (new Vector3(rotated.x, 0, rotated.y)) * Time.deltaTime * moveSpeed;

        Debug.DrawLine(transform.position, transform.position + (new Vector3(rotated.x, 0, rotated.y)).normalized * 3, Color.blue);
    }

    public Vector2 RotateVectorByDegrees(Vector2 orig, float degrees) { return RadToVector2(Vector2ToRad(orig) + (degrees * Mathf.Deg2Rad)) * orig.magnitude; }

    public float Vector2ToRad(Vector2 orig)
    {
        float angle;
        if (orig.x == 0)
        {
            if (orig.y < 0) { angle = 270 * Mathf.Deg2Rad; }
            else { angle = 90 * Mathf.Deg2Rad; }
        }
        
        else if (orig.y == 0)
        {
            if (orig.x < 0) { angle = 180 * Mathf.Deg2Rad; }
            else { angle = 0 * Mathf.Deg2Rad; }
        }

        else { angle = Mathf.Atan(orig.y / orig.x); if (orig.x < 0) { angle += Mathf.PI; } }

        return angle;
    }

    public Vector2 RadToVector2(float angle)
    {
        angle %= 360;
        float cos = Mathf.Cos(angle);
        return new Vector2(cos * (angle > 90 && angle < 270 ? -1 : 1), Mathf.Sin(angle));
    }

    public Vector3 NormalToRotation(Vector3 normal)
    {

        return Vector3.zero;
    }

    void NewZero(InputAction.CallbackContext context)
    {
        zero = rotation;
        Debug.Log("Zero: " + zero);
    }

    void sus()
    {
        RaycastHit sussy;
        Physics.Raycast(transform.position, Vector3.down, out sussy, 20);
        normalVector = sussy.normal;
    }
}