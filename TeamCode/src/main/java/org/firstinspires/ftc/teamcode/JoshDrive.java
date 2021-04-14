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
        //double power = Math.abs()
        double rightPower = gamepad1.left_stick_y * ((gamepad1.left_stick_x + 1)/ 2);

        //leftDrive.setPower(leftPower);


        telemetry.addData("Left", "X:" + gamepad1.left_stick_x + " Y:" + gamepad1.left_stick_y);
        telemetry.addData("Motors", "Left:" + leftDrive.getPower() + " Right:" + rightDrive.getPower());


    }
}
