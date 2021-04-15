package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TODO
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @version 1.0.0
 */

public class JoshDrive extends OpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = hardwareMap.get(DcMotor.class,"right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double rPower;
        double lPower;

        if (power == 0) {
            rPower = 0;
            lPower = 0;
        } else if (power > 0){ // Forwards
            rPower = Math.min(-turn+power,1);
            lPower = Math.min(turn+power,1);
        } else { // Backwards
            rPower = Math.max(turn-power,-1);
            lPower = Math.max(-turn-power,-1);
        }




        telemetry.addData("Left", "X:" + gamepad1.left_stick_x + " Y:" + gamepad1.left_stick_y);
        telemetry.addData("Motors", "Left:" + leftDrive.getPower() + " Right:" + rightDrive.getPower());


    }
}
