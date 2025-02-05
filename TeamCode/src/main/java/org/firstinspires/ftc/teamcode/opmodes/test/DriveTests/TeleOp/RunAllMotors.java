package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TEST AUTO", group="Linear OpMode")
public class RunAllMotors extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables for all motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set all motors to move in the same direction (FORWARD)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
      //  rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
      //  rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run all motors in a loop until the match ends
        while (opModeIsActive()) {
            // Set the motors to a constant power level (e.g., 0.5)
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);

            sleep(3000);

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Display run time and motor power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Right Power", rightFrontDrive.getPower());
            telemetry.addData("Front Left Power", leftFrontDrive.getPower());
            telemetry.addData("Back Right Power", rightBackDrive.getPower());
            telemetry.addData("Back Left Power", leftBackDrive.getPower());
            telemetry.update();
        }
    }
}