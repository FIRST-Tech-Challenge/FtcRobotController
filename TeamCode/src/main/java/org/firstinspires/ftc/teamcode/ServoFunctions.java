package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ServoFunctions {
    private LinearOpMode lom = null;
    private Servo pixelReleaseServo = null;
    static final double   PIXEL_RELEASE_SERVO_INIT_POS   = 0.7;
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 20;     // Larger is smoother, but potentially slower
    public ServoFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    private void Initialize()
    {
        pixelReleaseServo = lom.hardwareMap.get(Servo .class, "PixelReleaseServo");
        pixelReleaseServo.scaleRange(0.55, 1.0);
        pixelReleaseServo.setPosition(PIXEL_RELEASE_SERVO_INIT_POS);
    }
    public void PutPixelInBackBoard()
    {
        MoveServoSmoothly(pixelReleaseServo, 1.0, 1500);
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
    public void MovePixelReleaseServoRelative(double deltaMove)
    {
        pixelReleaseServo.setPosition(pixelReleaseServo.getPosition() + deltaMove);
    }
    public double GetPixelReleaseServoPosition()
    {
        return pixelReleaseServo.getPosition();
    }
}
