using System;
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

        float[] robotRot = new float[] { transform.forward.z, transform.forward.y, transform.forward.x };
        Vector3 glerp = new Vector3(robotRot[0], robotRot[1], robotRot[2]);

        Matrix matrix = new Matrix(transform.eulerAngles.x, transform.eulerAngles.y, transform.eulerAngles.z);
        float[] hehe = matrixMultiply(matrix.generalRotationMatrixExtrinsic, robotRot);
        Vector3 e = new Vector3(hehe[0], 0, hehe[2]);
        Debug.Log("robotRot: " + glerp + " | e: " + e);


        Debug.DrawLine(transform.position, transform.position + (Quaternion.Inverse(transform.rotation) * transform.forward) * 3, Color.black);
    }

    float[] balls = new float[] { 1f, 1f, 1f };

    public static float[] matrixMultiply(float[][] firstMatrix, float[] secondMatrix)
    {
        float[] newMatrix = new float[firstMatrix.GetLength(0)];
        if (firstMatrix.GetLength(0) == secondMatrix.GetLength(0))
        {
            for (int i = 0; i < 3; i++)
            {
                float currentRow = 0;
                for (int j = 0; j < 3; j++)
                {
                    currentRow += firstMatrix[i][j] * secondMatrix[j];
                }
                newMatrix[i] = currentRow;
            }

        }
        return newMatrix;
    }
}

public class Matrix
{
    public float[][] generalRotationMatrixIntrinsic;
    public float[][] generalRotationMatrixExtrinsic;

    public float alpha;
    public float beta;
    public float gamma;

    public Matrix(float x, float y, float z)
    {
        alpha = y;
        beta = x;
        gamma = z;

        CalculateMatrices();
    }

    public void CalculateMatrices()
    {
        generalRotationMatrixIntrinsic = new float[][] {new float[]{Mathf.Cos(alpha) * Mathf.Cos(beta), Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) - Mathf.Sin(alpha) * Mathf.Cos(gamma), Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) + Mathf.Sin(alpha) * Mathf.Sin(gamma)},
    new float[]{Mathf.Sin(alpha) * Mathf.Cos(beta), Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) + Mathf.Cos(alpha) * Mathf.Cos(gamma), Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) - Mathf.Cos(alpha) * Mathf.Sin(gamma)},
    new float[]{-Mathf.Sin(beta), Mathf.Cos(beta) * Mathf.Sin(gamma), Mathf.Cos(beta) * Mathf.Cos(gamma)}};

    generalRotationMatrixExtrinsic = new float[][] {new float[]{Mathf.Cos(beta) * Mathf.Cos(gamma), Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) - Mathf.Cos(alpha) * Mathf.Sin(gamma), Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) + Mathf.Sin(alpha) * Mathf.Sin(gamma)},
    new float[]{Mathf.Cos(beta) * Mathf.Sin(gamma), Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) + Mathf.Cos(alpha) * Mathf.Cos(gamma), Mathf.Cos(alpha) * Mathf.Sin(beta), Mathf.Sin(gamma) - Mathf.Sin(alpha) * Mathf.Cos(gamma)},
    new float[]{-Mathf.Sin(beta), Mathf.Sin(alpha) * Mathf.Cos(beta), Mathf.Cos(alpha) * Mathf.Cos(beta)}};
}
}

/*float xPos = Mathff.Sin(transform.eulerAngles.y * Mathff.Deg2Rad) * Mathff.Cos(transform.eulerAngles.x * Mathff.Deg2Rad);
        float yPos = Mathff.Sin(-transform.eulerAngles.x * Mathff.Deg2Rad);
        float zPos = Mathff.Cos(transform.eulerAngles.x * Mathff.Deg2Rad) * Mathff.Cos(transform.eulerAngles.y * Mathff.Deg2Rad);
*/