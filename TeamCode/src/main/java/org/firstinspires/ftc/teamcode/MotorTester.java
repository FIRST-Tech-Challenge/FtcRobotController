package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTester extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        telemetry.addData("Status", "Initialized!");
    }

    @Override
    public void loop() {

        telemetry.addData("Motors", "All Motors moving!");
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(1);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}

        telemetry.addData("Motors", "Front Left moving!");
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}

        telemetry.addData("Motors", "Front Right moving!");
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}

        telemetry.addData("Motors", "Back Left moving!");
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(0);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}

        telemetry.addData("Motors", "Back Right moving!");
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(1);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}

        telemetry.addData("Motors", "Stopped!");
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        try {   // Sleep for 2 seconds
            Thread.sleep(2000);
        } catch (InterruptedException ignored) {}
    }
}
