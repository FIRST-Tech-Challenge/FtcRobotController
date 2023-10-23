package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ServoFunctions {
    private LinearOpMode lom = null;
    private Servo pixelReleaseServo = null;
    private Servo shoulderServo = null;
    private Servo elbowServo = null;
    private Servo clawServo = null;
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 30;     // Larger is smoother, but potentially slower
    public ServoFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    private void Initialize()
    {
        pixelReleaseServo = lom.hardwareMap.get(Servo .class, "PixelReleaseServo");
        shoulderServo = lom.hardwareMap.get(Servo .class, "shoulder");
        elbowServo = lom.hardwareMap.get(Servo .class, "elbow");
        clawServo = lom.hardwareMap.get(Servo .class, "claw");

        pixelReleaseServo.scaleRange(0.35, 0.85);
        shoulderServo.scaleRange(0.35, 0.9494);
        elbowServo.scaleRange(0.1244, 0.8050);
        clawServo.scaleRange(0.35, 0.85);

        pixelReleaseServo.setPosition(0.0);
        shoulderServo.setPosition(0.0);
        elbowServo.setPosition(1.0);
        clawServo.setPosition(0.0);
    }
      public void PutPixelInBackBoard()
    {
        MoveServoSmoothly(pixelReleaseServo, 1.0, 800);
        lom.sleep(300);
        MoveServoSmoothly(pixelReleaseServo, 0.0, 500);
    }
    private void MoveServoSmoothly(Servo s, double endPosition, int timeInMilliseconds)
    {
        double stepSize = (endPosition - s.getPosition()) / SERVO_SMOOTH_MOVE_STEPS;
        long sleepTime = timeInMilliseconds / SERVO_SMOOTH_MOVE_STEPS;
        double position = s.getPosition();

        for (int i=0; i < SERVO_SMOOTH_MOVE_STEPS; i++)
        {
            position += stepSize;
            s.setPosition(position);
            lom.sleep(sleepTime);
        }
        s.setPosition(endPosition);
    }
    public void MoveShoulderRelative(double move)
    {
        MoveServoSmoothly(shoulderServo, shoulderServo.getPosition() + move, 10);
    }
    public void MoveElbowRelative(double move)
    {
        MoveServoSmoothly(elbowServo, elbowServo.getPosition() + move, 10);
    }
    public void MoveClawRelative(double move)
    {
        MoveServoSmoothly(clawServo, clawServo.getPosition() + move, 10);
    }

    public void MovePixelReleaseServoRelative(double move)
    {
        MoveServoSmoothly(pixelReleaseServo, pixelReleaseServo.getPosition() + move, 100);
    }
    public double GetPixelReleaseServoPosition()
    {
        return pixelReleaseServo.getPosition();
    }
    public double GetShoulderServoPosition()
    {
        return shoulderServo.getPosition();
    }
    public double GetElbowServoPosition()
    {
        return elbowServo.getPosition();
    }
    public double GetClawServoPosition()
    {
        return clawServo.getPosition();
    }
}
