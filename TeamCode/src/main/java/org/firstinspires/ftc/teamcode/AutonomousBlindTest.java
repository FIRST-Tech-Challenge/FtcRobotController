package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Drive Forward", group="Autonomous")
public class AutonomousBlindTest extends LinearOpMode {

    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        frontRight  = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        // Set the direction of the motors.
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY).
        waitForStart();

        // Set the power for both motors to drive forward.
        frontRight.setPower(0.5);
        frontLeft.setPower(0.5);
        backRight.setPower(0.5);
        backLeft.setPower(0.5);

        // Drive forward for one second.
        sleep(1000);

        // Stop the motors.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}