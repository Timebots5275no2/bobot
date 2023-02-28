using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class glop : MonoBehaviour
{
    [SerializeField] float pitch;
    [SerializeField] float roll;

    Vector3 orig;

    private void Awake()
    {
        orig = transform.eulerAngles;
    }

    void Update()
    {
        Debug.DrawLine(transform.position, transform.position + (transform.forward * 3), Color.green);

        Vector3 worldRot = transform.eulerAngles;
        pitch = worldRot.x;
        roll = worldRot.z;

        Vector2 sussyMcGlussy = Bobot.RadToVector2(transform.eulerAngles.y * Mathf.Deg2Rad);
        Vector3 dir = new Vector3(transform.eulerAngles.x, 0, transform.eulerAngles.z);

        Debug.DrawLine(transform.position, transform.position + (dir * 3), Color.blue);

        Vector3 sussyMcBussy = new Vector3(Mathf.Cos(pitch * Mathf.Deg2Rad), 0, Mathf.Sin(roll * Mathf.Deg2Rad));
        Debug.Log(sussyMcBussy);
        Debug.DrawLine(transform.position, transform.position + sussyMcBussy * 3 * -360, Color.cyan);
    }
}

/*float xPos = Mathf.Sin(transform.eulerAngles.y * Mathf.Deg2Rad) * Mathf.Cos(transform.eulerAngles.x * Mathf.Deg2Rad);
        float yPos = Mathf.Sin(-transform.eulerAngles.x * Mathf.Deg2Rad);
        float zPos = Mathf.Cos(transform.eulerAngles.x * Mathf.Deg2Rad) * Mathf.Cos(transform.eulerAngles.y * Mathf.Deg2Rad);
*/