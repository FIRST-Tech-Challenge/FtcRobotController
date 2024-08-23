package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vanilla", group="Robot")

public class RobotAutoDriveByTime_Linear extends LinearOpMode {

    // Initialize all variables for the program
    private IMU imu = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private final ElapsedTime     runtime = new ElapsedTime();
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;


    @Override
    public void runOpMode() {
        // Define all the hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Callie and Neha: write autonomous code here! - - - - - - - - - - - - - - - - - - - - - -
        moveForward(0.5);
        moveForward(0.5, 0.8);
        turnLeft(1);
        moveBackward(1.2);
        turnRight(.4);
        // End of autonomous program

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    private void moveForward(double secondsToDrive) {
        moveForward(secondsToDrive, FORWARD_SPEED);
    }

    private void moveForward(double secondsToDrive, double speedToDrive) {
        leftFrontDrive.setPower(speedToDrive);
        rightFrontDrive.setPower(speedToDrive);
        leftBackDrive.setPower(speedToDrive);
        rightBackDrive.setPower(speedToDrive);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            telemetry.addData("Path", "Move forward: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    private void moveBackward(double secondsToDrive) {
        moveBackward(secondsToDrive, FORWARD_SPEED);
    }

    private void moveBackward(double secondsToDrive, double speedToDrive) {
        leftFrontDrive.setPower(-speedToDrive);
        rightFrontDrive.setPower(-speedToDrive);
        leftBackDrive.setPower(-speedToDrive);
        rightBackDrive.setPower(-speedToDrive);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            telemetry.addData("Path", "Move backward: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    private void turnRight(double secondsToDrive) {
        turnRight(secondsToDrive, FORWARD_SPEED);
    }

    private void turnRight(double secondsToDrive, double speedToDrive) {
        leftFrontDrive.setPower(speedToDrive);
        rightFrontDrive.setPower(-speedToDrive);
        leftBackDrive.setPower(speedToDrive);
        rightBackDrive.setPower(-speedToDrive);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            telemetry.addData("Path", "Turn right: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    private void turnLeft(double secondsToDrive) {
        turnLeft(secondsToDrive, FORWARD_SPEED);
    }

    private void turnLeft(double secondsToDrive, double speedToDrive) {
        leftFrontDrive.setPower(-speedToDrive);
        rightFrontDrive.setPower(speedToDrive);
        leftBackDrive.setPower(-speedToDrive);
        rightBackDrive.setPower(speedToDrive);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            telemetry.addData("Path", "Turn left: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    private void stopMoving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
