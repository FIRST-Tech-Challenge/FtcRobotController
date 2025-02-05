package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//


@Autonomous(name="BlueRight RedLeft", group="Simple")
@Disabled
public class RedLeft extends LinearOpMode {

    // Declare motor variables
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Constants
    private static final double POWER = 1.0; // Motor power
    private static double INCHES_PER_SEC = 2.25;

    //left power = 0 moves left, right power = 0 moves right
    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions (reverse right motors for proper movement)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start
        waitForStart();

        // Move forward for a specified time
        frontLeftMotor.setPower(POWER);
        frontRightMotor.setPower(POWER);
        backLeftMotor.setPower(POWER);
        backRightMotor.setPower(POWER);

        // Sleep for the determined time to move forward 10 inches
        sleep(8890);

        // Stop all motion
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Status", "Task Complete");
        telemetry.update();
    }
}
