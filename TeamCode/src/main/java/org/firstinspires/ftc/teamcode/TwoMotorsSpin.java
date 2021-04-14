package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TODO
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @version 1.0.0
 */

@TeleOp
public class TwoMotorsSpin extends OpMode {
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
        double rightPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x - 1) / 2));
        double leftPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x + 1) / 2));

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);


        telemetry.addData("Left", "X:" + gamepad1.left_stick_x + " Y:" + gamepad1.left_stick_y);
        telemetry.addData("Motors", "Left:" + leftDrive.getPower() + " Right:" + rightDrive.getPower());


    }
}
