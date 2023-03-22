using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmPosClampTest : MonoBehaviour
{
    /*[SerializeField] GameObject groundConstraintVisualize;
    [SerializeField] GameObject pointVisualize;
    [Header("Map point settings")]
    [SerializeField] Transform startingArmPos;
    [SerializeField] Transform endPos;

    private void Start()
    {
        PlaceConstraints();
        MapToPoint(endPos.localPosition);
    }

    void PlaceConstraints()
    {
        foreach (Vector2 gc in ground_Constraints)
        {
            GameObject obj = Instantiate(groundConstraintVisualize, transform);
            obj.transform.localPosition = gc;
        }
    }

    void MapToPoint(Vector2 point)
    {
        List<Vector2> points = goToPoint(point);

        foreach(Vector2 p in points)
        {
            GameObject obj = Instantiate(pointVisualize, transform);
            obj.transform.localPosition = p;
        }
    }



    Vector2 GetConstraintsBetween(Vector2 pos)
    {
        for (int i = 0; i < ground_Constraints.Length; i++)
        {
            if (ground_Constraints[i].x < pos.x && i + 1 < ground_Constraints.Length && ground_Constraints[i + 1].x > pos.x)
            {
                return new Vector2(i, i + 1);
            }
        }

        return new Vector2(-1, -1);
    }

    public List<Vector2> goToPoint(Vector2 pointToGoTo) // for creating move sequence
    {
        Vector2 armPos = startingArmPos.localPosition;
        double inchesBetweenPoints = Vector2.Distance(armPos, pointToGoTo);
        int totalPoints = (int)Math.Ceiling(inchesBetweenPoints * Move_Points_Per_Inch);
        List<Vector2> moveSequence = new List<Vector2>();

        for (int i = 0; i <= totalPoints; i++)
        {
            moveSequence.Add(GetClampedPosValue(Vector2.Lerp(armPos, pointToGoTo, i / (float)totalPoints)));
        }

        return moveSequence;
    }

    public float PercentBetweenNumbers(float value, float min, float max)
    {
        float offset = 0 - min;
        return (value + offset) / (max + offset);
    }

    public static Vector2[] ground_Constraints = { 
        new Vector2(-30.0061f, -25.8670f), //  Back of motor
        new Vector2(-26.5601f, -26.117f), //  Far left top of battery mount
        new Vector2(-23.398f, -16.5532f), //  Left top arm mount
        new Vector2(-22.6487f, -15.424f), //  Left motor
        new Vector2(-21.0854f, -13.8517f), //  Top motor
        new Vector2(-19.1559f, -15.7821f), //  Right motor
        new Vector2(-17.0811f, -22.5870f), //  Right battery
        new Vector2(6.9939f, -25.8670f), //  Front of bumper
    };

    public static float gripperRadius = 3; // inches
    public static float Move_Points_Per_Inch = 1;*/
}
