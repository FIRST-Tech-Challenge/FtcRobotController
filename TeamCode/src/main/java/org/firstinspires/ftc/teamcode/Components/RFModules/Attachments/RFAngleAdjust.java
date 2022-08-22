package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Robot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFAngleAdjust extends RFDualServo{
    private RFDualServo angleAdjustServo;
    private double lastTime=0, lastServoPos=0;
    private final double DEG_PER_TICK_SERVO = 118.0/270.0/35.0, minDiffTime = 0.3;

    LinearOpMode op;
    public RFAngleAdjust(Servo.Direction servoDirection, LinearOpMode opMode) {
        super(servoDirection, opMode);

        angleAdjustServo = new RFDualServo(servoDirection, opMode);

        op = opMode;
    }

    public void TurretAngleControlRotating (double torget_point) { //pog
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

        if(thisTime-lastTime>minDiffTime) {
            angleAdjustServo.setPosition(torget_point);
            lastTime=thisTime;
            lastServoPos=torget_point;
        }

    }

    public void AutoAngleControlRotating (double torget_point) { //pog
        torget_point*=DEG_PER_TICK_SERVO;
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

        if(thisTime-lastTime>minDiffTime) {
            angleAdjustServo.setPosition(torget_point);
            lastTime=thisTime;
            lastServoPos=torget_point;
            op.telemetry.addData("lastTime",lastTime);
        }

    }
}
