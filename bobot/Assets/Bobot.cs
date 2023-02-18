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

    private void Start()
    {
        input = new Input();
        input.Yourmother.Enable();
    }

    private void Update()
    {
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

    void Rotate() { transform.eulerAngles = Vector3.up * rotation; }

    void Drive(Vector2 direction, float rot)
    {
        if (direction.magnitude > 0)
        {
            Vector2 rot2D = new Vector2(transform.forward.x, transform.forward.z);
            Vector2 rotated = RotateVector(direction, (Vector2ToRad(rot2D) * Mathf.Rad2Deg) + rot);
            Debug.Log(rotated);
            transform.position += (new Vector3(rotated.x, 0, rotated.y)) * Time.deltaTime * moveSpeed;

            Debug.DrawLine(transform.position, transform.position + (new Vector3(rotated.x, 0, rotated.y)).normalized * 3, Color.blue);
        }
    }

    public Vector2 RotateVector(Vector2 orig, float rotation) { return RadToVector2(Vector2ToRad(orig)) * orig.magnitude; }

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
}
