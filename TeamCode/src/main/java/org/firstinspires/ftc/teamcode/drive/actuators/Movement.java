package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class Movement extends OpMode {
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    Servo lright;
    Servo lleft;
    @Override
    public void init() {
        lright = hardwareMap.get(Servo.class,"lright");
        lleft = hardwareMap.get(Servo.class,"lleft");

        lright.setPosition(1);
        lleft.setPosition(0.1);

        backLeft = hardwareMap.dcMotor.get("odor");
        backRight = hardwareMap.dcMotor.get("odom");
        frontLeft = hardwareMap.dcMotor.get("odol");
        frontRight = hardwareMap.dcMotor.get("FR");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reversão de valores
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hardware: ", "Initialized");
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        anda();
    }
    private void anda(){
        double forward = gamepad2.left_stick_y;
        double strafe = -gamepad2.left_stick_x;
        double turn = -gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.5);

        frontRight.setPower((forward - strafe - turn) / denominator);
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }
}