package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;

public class RFAngleAdjust extends RFDualServo{
    private RFDualServo angleAdjustServo;
    private double lastTime=0, lastServoPos=0;
    private final double DEG_PER_TICK_SERVO = 118.0/270.0/35.0, minDiffTime = 0.3;

    LinearOpMode op;
    public RFAngleAdjust(String deviceName1, String deviceName2, double limit) {
        super(deviceName1, deviceName2, limit);

        angleAdjustServo = new RFDualServo(deviceName1, deviceName2, limit);

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

    public void AngleControlRotating (double torget_point) {
        torget_point*=DEG_PER_TICK_SERVO;
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }
//        turret_Angle_Control.setPosition(torget_point); current dual servo set position does not work with unique parameters
//        turret_Angle_Control2.setPosition(118.0/270-torget_point);
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
}
