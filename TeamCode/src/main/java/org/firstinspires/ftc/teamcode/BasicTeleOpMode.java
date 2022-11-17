package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BasicTeleOpMode extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    // DcMotor armMotor;
    // Servo coneClaw;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");
        
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        // coneClaw = hardwareMap.get(Servo.class, "Claw");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        double left  = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
    }
}
