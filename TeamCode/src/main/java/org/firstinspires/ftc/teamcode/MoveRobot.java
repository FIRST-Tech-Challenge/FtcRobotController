package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.concurrent.TimeUnit;

@Autonomous
public class MoveRobot extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        // Initialize motors and other components
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeftWheel");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRightWheel");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeftWheel");
        backRightMotor = hardwareMap.dcMotor.get("BackRightWheel");
        ElapsedTime runtime = new ElapsedTime();

        // Set the direction of the right side motors to reverse
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the start button to be pressed on the driver station
        waitForStart();

        // Move the robot forward for 2 seconds
        moveForward();

        // Stop the robot
        stopRobot();
    }

    private void moveForward() {
        // Set power to all four motors
        frontRightMotor.setPower(0.482);
        frontLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.482);
        backLeftMotor.setPower(0.5);

        // Sleep for the specified duration
        try {
            Thread.sleep(500, TimeUnit.MILLISECONDS.compareTo(TimeUnit.MILLISECONDS));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    private void stopRobot() {
        // Stop all four motors
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }}