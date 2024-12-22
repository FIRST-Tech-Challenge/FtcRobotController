package org.firstinspires.ftc.teamcode.Systems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Drivetrain {

    Hardware hardware;

    private DcMotorEx LF;
    private DcMotorEx RF;
    private DcMotorEx RB;
    private DcMotorEx LB;

    private GoBildaPinpointDriver pinPoint;

    private double[] sticks = {0, 0, 0, 0};
    private double heading = 0;

    GamepadEx gamepad;


    public Drivetrain(Hardware hardware, GamepadEx gamepad) {
        this.hardware = hardware;

        LF = hardware.LF;
        RF = hardware.RF;
        RB = hardware.RB;
        LB = hardware.LB;

        pinPoint = hardware.pinPoint;
        this.gamepad = gamepad;
    }

    public void update() {
        pinPoint.update();
        heading = pinPoint.getHeading();

        sticks[0] = -gamepad.getRightX();
        sticks[1] = gamepad.getRightY();
        sticks[2] = -gamepad.getLeftX();
        sticks[3] = gamepad.getLeftY();
    }

    public void command() {
        double rotX = sticks[2] * Math.cos(heading) - sticks[3] * Math.sin(heading);
        double rotY = sticks[2] * Math.sin(heading) + sticks[3] * Math.cos(heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(sticks[0]), 1);
        double LFPower = (rotY + rotX + sticks[0]) / denominator;
        double RFPower = (rotY - rotX - sticks[0]) / denominator;
        double RBPower = (rotY + rotX - sticks[0]) / denominator;
        double LBPower = (rotY - rotX + sticks[0]) / denominator;

        LF.setPower(LFPower);
        RF.setPower(RFPower);
        RB.setPower(RBPower);
        LB.setPower(LBPower);
    }

    public Pose2D getPos2D() {
        return  pinPoint.getPosition();
    }

}
