using UnityEngine;

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
    float secondJointLength = 34.5f;
    TwoJointInverseKinematics kinematics;
    Input input;

    readonly float gripperRadius = 7; // inches
    readonly float Move_Sequence_Allowed_Error = 0.5f;
    readonly float Move_Points_Per_Inch = 1;

    readonly float farthestBackChassisPos = -14; // On chart: 1, 2
    readonly float frontOfBumperXPos = 7; // On chart: 
    readonly float frontGroundXPos = 15;
    readonly float insideChassisLargestY = -16.5f;
    readonly float insideChassisSmallestY = -21f;
    readonly float outsideChassisSmallestY = -27;

    readonly float ARM_FIRST_PART_LENGTH = 18;
    readonly float ARM_SECOND_PART_LENGTH = 34.5f; /*For some reason the X is typically +1 */
    readonly float FARTHEST_EXTENSION_POINT = 48;

    private void Start()
    {
        kinematics = new TwoJointInverseKinematics(firstJointLength, secondJointLength);
        input = new Input();
        input.Yourmother.MousePos.Enable();
        input.Yourmother.Movement.Enable();
    }

    private void Update()
    {
        GetMouseInput();
        //GetAnalogInput();
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
        Vector2 o = pos;
        float armDiff = Mathf.Abs(ARM_FIRST_PART_LENGTH - ARM_SECOND_PART_LENGTH);
        float farthestArmReach = kinematics.totalDistance();

        if (o.x < frontOfBumperXPos) // Inside chassis
        {
            if (o.y > insideChassisLargestY && o.x < 0) { o.y = insideChassisLargestY; }
            else if (o.y < insideChassisSmallestY) { o.y = insideChassisSmallestY; }
        }
        else // Outside chassis
        {
            float p = PercentBetweenNumbers(o.x, frontOfBumperXPos, frontGroundXPos);
            Vector2 point3 = new Vector2(frontOfBumperXPos, insideChassisSmallestY);
            Vector2 point4 = new Vector2(frontGroundXPos, outsideChassisSmallestY);
            o.y = clampNumber(o.y, Vector2.Lerp(point3, point4, p).y, farthestArmReach);

            if (o.y < outsideChassisSmallestY) { o.y = outsideChassisSmallestY; }
        }

        if (o.y >= 0 && o.x < armDiff) { o.x = armDiff; } // Above y=0

        if (o.magnitude < armDiff + .5f) { o = o.normalized * (armDiff + .5f); } // Clamp outside min circle
        o = Vector2.ClampMagnitude(o, farthestArmReach - .5f); // Clamp inside max circle
        o.x = clampNumber(o.x, farthestBackChassisPos, FARTHEST_EXTENSION_POINT); // Clamping between smallest and largest allowed x value

        return o;
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

    void RenderDebug()
    {
        float armDiff = Mathf.Abs(ARM_FIRST_PART_LENGTH - ARM_SECOND_PART_LENGTH);
        float dist = ARM_FIRST_PART_LENGTH + ARM_SECOND_PART_LENGTH;

        Vector2 one = new Vector2(farthestBackChassisPos, insideChassisLargestY);
        Vector2 two = new Vector2(farthestBackChassisPos, insideChassisSmallestY);
        Vector2 three = new Vector2(frontOfBumperXPos, insideChassisSmallestY);
        Vector2 four = new Vector2(frontGroundXPos, outsideChassisSmallestY);
        Vector2 five = new Vector2(FARTHEST_EXTENSION_POINT, outsideChassisSmallestY);
        Vector2 six = new Vector2(FARTHEST_EXTENSION_POINT, dist);
        Vector2 seven = new Vector2(armDiff, dist);
        Vector2 eight = new Vector2(armDiff, 0);
        Vector2 nine = new Vector2(0, -armDiff);

        Debug.DrawLine(one, two, Color.magenta);
        Debug.DrawLine(two, three, Color.magenta);
        Debug.DrawLine(three, four, Color.magenta);
        Debug.DrawLine(four, five, Color.magenta);
        Debug.DrawLine(five, six, Color.magenta);
        Debug.DrawLine(six, seven, Color.magenta);
        Debug.DrawLine(seven, eight, Color.magenta);
        Debug.DrawLine(nine, one, Color.magenta);
    }

    private void OnDrawGizmos()
    {
        RenderDebug();
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(firstJointEncoder.position, Mathf.Abs(ARM_FIRST_PART_LENGTH - ARM_SECOND_PART_LENGTH)); // Min circle
        Gizmos.DrawWireSphere(firstJointEncoder.position, ARM_FIRST_PART_LENGTH + ARM_SECOND_PART_LENGTH); // Max circle
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
