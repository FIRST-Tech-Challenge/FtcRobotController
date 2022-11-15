package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpMain extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor armMotor;
    Servo coneGrabber;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        coneGrabber = hardwareMap.get(Servo.class, "Grabber");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // The following code will allow for the driver to control the robot's direction, strafe, and rotation
        // https://github.com/brandon-gong/ftc-mecanum
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for (double currentMotor : speeds) {
            if (max < Math.abs(currentMotor)) {
                max = Math.abs(currentMotor);
            }
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeftMotor.setPower(speeds[0]);
        frontRightMotor.setPower(speeds[1]);
        backLeftMotor.setPower(speeds[2]);
        backRightMotor.setPower(speeds[3]);


        // This code controls the arm!
        // Close the grabber
        if (gamepad1.a) {
            coneGrabber.setPosition(1);
        }
        // Open the grabber
        if (gamepad1.x) {
            coneGrabber.setPosition(0);
        }

        // Raise the arm
        if (gamepad1.y) {
            armMotor.setPower(0.5);
        }
        // Lower the arm
        if (gamepad1.b) {
            armMotor.setPower(-0.5);
        }
    }
}