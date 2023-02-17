using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Bobot : MonoBehaviour
{
    [SerializeField] float moveSpeed;

    Input input;
    Vector2 movementVal;

    private void Start()
    {
        input = new Input();
        input.Yourmother.Enable();
    }

    private void Update()
    {
        GetInput();
    }

    void GetInput()
    {
        movementVal = input.Yourmother.Movement.ReadValue<Vector2>().normalized * moveSpeed * Time.deltaTime;
    }

    void Drive(Vector2 direction, float rot)
    {
        transform.position += transform.forward * 
    }
}
