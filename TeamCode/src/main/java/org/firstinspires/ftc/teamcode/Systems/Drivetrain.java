package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Drivetrain {

    Hardware hardware;

    private DcMotorEx LF;
    private DcMotorEx RF;
    private DcMotorEx RB;
    private DcMotorEx LB;

    private GoBildaPinpointDriver pinPoint;

    // Controller Joystick coefficients | {rx, ry, lx, ly}
    final private double[] pS4 = {1, -1, 1, -1};
    final private double[] xBox = {-1, -1, -1, -1};
    private double[] stickCoefficients = {0, 0, 0, 0};

    // Joystick Positions | {rx, ry, lx, ly}
    private double[] sticks = {0, 0, 0, 0};

    public enum Controller {
        xBox,
        pS4
    }


    Gamepad gamepad;


    public Drivetrain(Hardware hardware, Gamepad gamepad, Controller controller) {
        this.hardware = hardware;

        LF = hardware.LF;
        RF = hardware.RF;
        RB = hardware.RB;
        LB = hardware.LB;

        pinPoint = hardware.pinPoint;

        switch(controller) {
            case xBox:
                stickCoefficients = xBox;
                break;
            case pS4:
                stickCoefficients = pS4;
                break;
        }

        this.gamepad = gamepad;
    }

    public void update() {

        sticks[0] = gamepad.right_stick_x;
        sticks[1] = gamepad.right_stick_y;
        sticks[2] = gamepad.left_stick_x;
        sticks[3] = gamepad.left_stick_y;

        for (int i = 0; i < sticks.length; i++) {
            sticks [i] *= stickCoefficients[i];
        }

        double heading = -1 * pinPoint.getHeading();

        double rotX = sticks[2] * Math.cos(-heading) - sticks[3] * Math.sin(-heading);
        double rotY = sticks[2] * Math.sin(-heading) + sticks[3] * Math.cos(-heading);

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
}
