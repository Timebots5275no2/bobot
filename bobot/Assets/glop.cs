using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class glop : MonoBehaviour
{
    [SerializeField] float pitch;
    [SerializeField] float roll;

    Vector3 orig;

    void Update()
    {
        Debug.DrawLine(transform.position, transform.position + (transform.forward * 3), Color.green);

        Matrix posMatrix = Matrix.PositionMatrix(Vector3.forward);
        Matrix rotationMatrix = Matrix.RotationMatrix(transform.eulerAngles * Mathf.Deg2Rad);
        Matrix rotated = rotationMatrix * posMatrix;

        Debug.DrawLine(transform.position, transform.position + (Vector3)rotated * 3, Color.red);
    }

}

public class Matrix
{
    public float[][] matrix;

    public Matrix(float[][] matrix)
    {
        this.matrix = matrix;
    }

    public static Matrix operator *(Matrix one, Matrix two)
    {
        Matrix newMatrix = new Matrix(ArrayHelper.FloatArray(one.matrix.Length, two.matrix[0].Length));

        for (int i = 0; i < newMatrix.matrix.Length; i++)
        {
            for (int j = 0; j < newMatrix.matrix[i].Length; j++)
            {
                newMatrix.matrix[i][j] = SussyDotProduct(one.matrix[i], two.arrayFromColumn(j));
            }
        }

        return newMatrix;
    }

    public float[] arrayFromColumn(int col)
    {
        float[] output = new float[matrix.Length];

        for (int i = 0; i < output.Length; i++)
        {
            output[i] = matrix[i][col];
        }

        return output;
    }

    public static float SussyDotProduct(float[] one, float[] two)
    {
        float total = 0;
        for (int i = 0; i < one.Length; i++) { total += one[i] * two[i]; }
        return total;
    }

    public static Matrix RotationMatrix(Vector3 rad)
    {
        Matrix rotMat = new Matrix(ArrayHelper.FloatArray(3, 3));
        float alpha = rad.z;
        float beta = rad.y;
        float gamma = rad.x;

        rotMat.matrix[0][0] = Mathf.Cos(alpha) * Mathf.Cos(beta);
        rotMat.matrix[0][1] = Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) - (Mathf.Sin(alpha) * Mathf.Cos(gamma));
        rotMat.matrix[0][2] = Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) + (Mathf.Sin(alpha) * Mathf.Sin(gamma));

        rotMat.matrix[1][0] = Mathf.Sin(alpha) * Mathf.Cos(beta);
        rotMat.matrix[1][1] = Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) + (Mathf.Cos(alpha) * Mathf.Cos(gamma));
        rotMat.matrix[1][2] = Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Cos(gamma) - (Mathf.Cos(alpha) * Mathf.Sin(gamma));

        rotMat.matrix[2][0] = -Mathf.Sin(beta);
        rotMat.matrix[2][1] = Mathf.Cos(beta) * Mathf.Sin(gamma);
        rotMat.matrix[2][2] = Mathf.Cos(beta) * Mathf.Cos(gamma);

        return rotMat;
    }

    public static Matrix PositionMatrix(Vector3 pos)
    {
        float[][] sus = new float[][] { new float[] { pos.x }, new float[] { pos.y }, new float[] { pos.z } };
        return new Matrix(sus);
    }

    public override string ToString()
    {
        string output = "";

        for (int i = 0; i < matrix.Length; i++)
        {
            output += "[";
            for (int j = 0; j < matrix[i].Length; j++)
            {
                output += matrix[i][j] + (j + 1 == matrix[i].Length ? "" : ", ");
            }
            output += "]\n";
        }

        return output;
    }

    public static explicit operator Vector3(Matrix matrix)
    {
        return new Vector3(matrix.matrix[0][0], matrix.matrix[1][0], matrix.matrix[2][0]);
    }
}

public static class ArrayHelper
{
    public static float[][] FloatArray(int rows, int cols)
    {
        float[][] sus = new float[rows][];

        for (int i = 0; i < sus.Length; i++)
        {
            sus[i] = new float[cols];
        }

        return sus;
    }
}