using UnityEngine;
using UnityEngine.Windows;
using static UnityEditor.PlayerSettings;

public class KinematicsController : MonoBehaviour
{
    [SerializeField] Transform targetPoint;
    [SerializeField] Transform firstJointEncoder;
    [SerializeField] Transform secondJointEncoder;
    [SerializeField] Camera refCam;
    [Header("Values")]
    [SerializeField] Vector3 mouseScreenPos;
    [SerializeField] Vector3 mouseWorldPos;
    [SerializeField] Vector3 analogInput;
    [SerializeField] float analogSpeed;

    float firstJointLength = 18;
    float secondJointLength = 28;
    TwoJointInverseKinematics kinematics;
    Input input;

    float farthestBackX = -14;
    float frontBumperX = 7f;
    float frontGroundPosX = 19.5f;
    float farthestUpYOverRobot = -9f;
    float farthestDownYOverRobot = -19.5f;
    float farthestDownYOverGround = -28f;

    private void Start()
    {
        kinematics = new TwoJointInverseKinematics(firstJointLength, secondJointLength);
        input = new Input();
        input.Yourmother.MousePos.Enable();
        input.Yourmother.Movement.Enable();
    }

    private void Update()
    {
        //GetMouseInput();
        GetAnalogInput();
        targetPoint.position = GetClampedPosValue(targetPoint.position);

        MoveArm(targetPoint.position);
    }

    void GetAnalogInput()
    {
        analogInput = input.Yourmother.Movement.ReadValue<Vector2>();

        Vector2 scaledInput = analogInput * analogSpeed * Time.deltaTime;

        targetPoint.position += (Vector3)scaledInput;
    }

    void GetMouseInput()
    {
        mouseScreenPos = input.Yourmother.MousePos.ReadValue<Vector2>(); mouseScreenPos.z = 32;
        mouseWorldPos = refCam.ScreenToWorldPoint(mouseScreenPos);
        targetPoint.position = mouseWorldPos;
    }

    public Vector2 GetClampedPosValue(Vector2 pos)
    {
     float gripperRadius = 7; // inches
     float Move_Sequence_Allowed_Error = 0.5f;
     float Move_Points_Per_Inch = 1;

     float farthestBackTargetPos = -14;
     float bumperFrontXPos = 7;
     float frontXPos = 20;
     float robotLargestY = -9;
     float robotSmallestY = -19.5f;
     float groundSmallestY = -27;

     float ARM_FIRST_PART_LENGTH = 18;
     float ARM_SECOND_PART_LENGTH = 28; /*For some reason the X is typically +1 */
     float FARTHEST_EXTENSION_POINT = 48;

    Vector2 o = pos;

        if (o.x<farthestBackTargetPos) { o.x = farthestBackTargetPos; }

if (o.x < bumperFrontXPos) // Inside robot
{
    o.y = clampNumber(o.y, robotSmallestY, robotLargestY);
}
else if (o.x < frontXPos) // Within front bounds
{
    float p = PercentBetweenNumbers(o.x, bumperFrontXPos, frontXPos);

    Vector2 bumperDownMax = new Vector2(bumperFrontXPos, robotSmallestY);
    Vector2 bumperUpMax = new Vector2(bumperFrontXPos, robotLargestY);

    Vector2 oDownMax = new Vector2(frontXPos, groundSmallestY);
    Vector2 oUpMax = new Vector2(frontXPos, kinematics.totalDistance());

    o.y = clampNumber(o.y, Vector2.Lerp(bumperDownMax, oDownMax, p).y, Vector2.Lerp(bumperUpMax, oUpMax, p).y);
}
else // oside of front bounds
{
    o.y = clampNumber(o.y, groundSmallestY, kinematics.totalDistance());
}

        float theNumber = Mathf.Abs(ARM_FIRST_PART_LENGTH - ARM_SECOND_PART_LENGTH) + .5f;
if (o.magnitude < theNumber) { o = o.normalized * theNumber; }
o = Vector2.ClampMagnitude(o, ARM_FIRST_PART_LENGTH + ARM_SECOND_PART_LENGTH - .5f);
o.x = clampNumber(o.x, farthestBackTargetPos, FARTHEST_EXTENSION_POINT);
return o;

        /*Vector2 clampedPos = new Vector2(pos.x, pos.y);

        Vector2 betweenIndexs = GetConstraintsBetween(pos); // x is first index, y is second index
        Vector2 diffBetweenPoints = ArmConstants.ground_Constraints[(int)betweenIndexs.x].substract(ArmConstants.ground_Constraints[(int)betweenIndexs.y]);
        double percentBetweenIndexs = PercentBetweenNumbers(pos.x, ArmConstants.ground_Constraints[(int)betweenIndexs.x].x, ArmConstants.ground_Constraints[(int)betweenIndexs.y].x);
        Vector2 clampPos = Vector2.lerp(ArmConstants.ground_Constraints[(int)betweenIndexs.x], ArmConstants.ground_Constraints[(int)betweenIndexs.y], percentBetweenIndexs); // Position at the percent (lerped)

        if (Vector2.distance(pos, clampPos) <= ArmConstants.gripperRadius || clampPos.y > pos.y + ArmConstants.gripperRadius) { clampedPos = clampPos.add(Vector2.clampMagnitude(new Vector2(diffBetweenPoints.y > 0 ? -1 : 1, 1), 1).times(ArmConstants.gripperRadius)); }
        return Vector2.clampMagnitude(clampedPos, kinematics.totalDistance()); // Clamps  value to total distance of the arm
      }

      Vector2 GetConstraintsBetween(Vector2 pos) {
        for (int i = 0; i < ArmConstants.ground_Constraints.length; i++) {
          if (ArmConstants.ground_Constraints[i].x < pos.x && i + 1 < ArmConstants.ground_Constraints.length && ArmConstants.ground_Constraints[i + 1].x > pos.x) {
            return new Vector2(i, i + 1);
          }
        }

        return new Vector2(-1, -1);*/
    }

    float clampNumber(float val, float min, float max)
    {
        if (val < min) { return min; }
        else if (val > max) { return max; }
        return val;
    }

    void MoveArm(Vector2 pos)
    {
        firstJointEncoder.localEulerAngles = Vector3.forward * kinematics.solveFirstJoint(pos);
        secondJointEncoder.localEulerAngles = Vector3.forward * kinematics.solveSecondJoint(pos);
    }

    public Vector2 RotateVectorByDegrees(Vector2 orig, float degrees) { return RadToVector2(Vector2ToRad(orig) + (degrees * Mathf.Deg2Rad)) * (orig.magnitude); }

    public static float Vector2ToRad(Vector2 orig)
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
            else { angle = 0; }
        }

        else { angle = Mathf.Atan(orig.y / orig.x); if (orig.x < 0) { angle += Mathf.PI; } }

        return angle;
    }

    public static Vector2 RadToVector2(float angle)
    {
        angle %= 360;
        float cos = Mathf.Cos(angle);
        return new Vector2(cos * (angle > 90 && angle < 270 ? -1 : 1), Mathf.Sin(angle));
    }

    public float PercentBetweenNumbers(float value, float min, float max)
    {
        float offset = 0 - min;
        return (value + offset) / (max + offset);
    }
}

public class TwoJointInverseKinematics
{
    private float firstJointLength;
    private float secondJointLength;

    public TwoJointInverseKinematics(float firstLength, float secondLength)
    {
        firstJointLength = firstLength;
        secondJointLength = secondLength;
    }

    public float solveFirstJoint(Vector2 target)
    {
        float distance = Vector2.Distance(Vector2.zero, target);
        if (target.x >= 0)
        {
            return ((Mathf.Atan(target.y / target.x) - lawOfCosines(firstJointLength, distance, secondJointLength)) * Mathf.Rad2Deg);
        }
        else
        {
            return (180 + (Mathf.Atan(target.y / target.x) - lawOfCosines(firstJointLength, distance, secondJointLength)) * Mathf.Rad2Deg);
        }
    }

    public float solveSecondJoint(Vector2 target)
    {
        float distance = Vector2.Distance(Vector2.zero, target);
        return 180 - lawOfCosines(firstJointLength, secondJointLength, distance) * Mathf.Rad2Deg;
    }

    public float lawOfCosines(float leg1, float leg2, float hypotenuse)
    {
        return Mathf.Acos((Mathf.Pow(leg1, 2) + Mathf.Pow(leg2, 2) - Mathf.Pow(hypotenuse, 2)) / (2 * leg1 * leg2));
    }

    public float totalDistance() { return firstJointLength + secondJointLength; }
}
