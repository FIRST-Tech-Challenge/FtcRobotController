package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Vanilla", group="Robot")

public class RobotAutoDriveByTime_Linear extends LinearOpMode {
    // Initialize all variables for the program
    // Hardware variables
    private IMU imu = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    // Software variables
    private final ElapsedTime     runtime = new ElapsedTime();
    static final double     DEFAULT_SPEED = 0.6;

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
        imu.resetYaw();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.update();
        sleep(5000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start of autonomous program
        /*
       turnLeft(2);
       turnRight(0.3);
       moveBackward(0.5);
       moveForward(0.5);
       moveBackward(0.5);
       moveForward(0.5);
       turnLeft(0.7);
       turnRight(0.7);
       for(int i=0; i < 2; i++)
        {
            turnLeft(2);
            moveForward(0.3);
            moveBackward(0.3);
            turnRight(2);
        }

       turnLeftToHeading(90, .2);
       sleep(2000);
       turnRightToHeading(0, .2);
       sleep(2000);
       turnLeftToHeading(90, .5);
       sleep(2000);
       turnRightToHeading(0, .5);
       sleep(2000);
       turnLeftToHeading(90, .7);
       sleep(2000);
       turnRightToHeading(0, .7);
       sleep(2000);
       turnLeftToHeading(90, .9);
       sleep(2000);
       turnRightToHeading(0, .9);
       sleep(5000);
*/
        moveForward(1, 0.8);
        sleep(1000);
        turnLeftToHeading(179, .2);
        moveForward(1, 0.2);
        sleep(1000);
        turnLeftToHeading(179, .2);

        // End of autonomous program

        telemetry.addData("Path", "Complete");
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.update();
        sleep(5000);
    }
    private void acceleration(double secondsToDrive, double speedToDrive, double leftDriveDirection, double rightDriveDirection) {
        double targetSpeed = speedToDrive; // Store the original target speed
        double currentSpeed = 0.0;

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            double elapsedTime = runtime.seconds();

            // Acceleration phase
            if (elapsedTime < 1 && currentSpeed < targetSpeed) {
                currentSpeed = currentSpeed + 0.01; // Increase the speed by 0.01 per second
            }
            // Deceleration phase
            else if (elapsedTime > secondsToDrive - 1 && elapsedTime < secondsToDrive && currentSpeed > 0) {
                currentSpeed = currentSpeed - 0.01; // Decrease the speed by 0.01 per second
            }

            leftFrontDrive.setPower(currentSpeed*leftDriveDirection);
            rightFrontDrive.setPower(currentSpeed*rightDriveDirection);
            leftBackDrive.setPower(currentSpeed*leftDriveDirection);
            rightBackDrive.setPower(currentSpeed*rightDriveDirection);

            telemetry.addData("Move forward: %4.1f S Elapsed", runtime.seconds());
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

    private void moveForward(double secondsToDrive) {
        moveForward(secondsToDrive, DEFAULT_SPEED);
    }

    private void moveForward(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, 1);
    }

    private void moveBackward(double secondsToDrive) {
        moveBackward(secondsToDrive, DEFAULT_SPEED);
    }

    private void moveBackward(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, -1);
    }

    private void turnLeft(double secondsToDrive) {
        turnLeft(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnLeft(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, 1);
    }

    private void turnRight(double secondsToDrive) {
        turnRight(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnRight(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, -1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void turnLeftToHeading(double targetYaw, double speedToDrive) {
        leftFrontDrive.setPower(-speedToDrive);
        rightFrontDrive.setPower(speedToDrive);
        leftBackDrive.setPower(-speedToDrive);
        rightBackDrive.setPower(speedToDrive);

        while (getHeading() < targetYaw) {
            telemetry.addData("Current Yaw", "%.0f", getHeading());
            telemetry.update();
        }
        stopMoving();
    }

    private void turnRightToHeading(double targetYaw, double speedToDrive) {
        leftFrontDrive.setPower(speedToDrive);
        rightFrontDrive.setPower(-speedToDrive);
        leftBackDrive.setPower(speedToDrive);
        rightBackDrive.setPower(-speedToDrive);

        while (getHeading() > targetYaw) {
            telemetry.addData("Current Yaw", "%.0f", getHeading());
            telemetry.update();
        }
        stopMoving();
    }
}