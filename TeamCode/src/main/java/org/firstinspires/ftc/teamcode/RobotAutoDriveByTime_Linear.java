package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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
    private Servo leftArm = null;
    private static final double ARM_DEFAULT = 0.3;
    private static final double ARM_MIN = 0.0;
    private static final double ARM_MAX = 1.0;

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
        //leftArm = hardwareMap.get(Servo.class, "claw");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //double arm_position = ARM_DEFAULT;
        //leftArm.setPosition(arm_position);

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
        //sleep(5000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

/*
        moveForward(1.2, 0.8);
        strafeLeft(0.5, 0.8);
        moveBackward(0.6);
        turnRightToHeading(-3, 0.8);
        moveBackward(1.7, 0.4);
        moveForward(0.4, 0.8);
        turnRightToHeading(-1,0.5);
        moveForward(0.6, 0.8);
        turnLeftToHeading(0.5, 0.3);
        strafeLeft(0.9, 0.4);
        moveBackward(1.4, 0.4);
        */

        moveForward(2.3);
        strafeLeft(0.5); //strafing to 1st block
        moveBackward(2.1,0.4 ); //moving first block backward
        turnRightToHeading(-20, 0.4);//turning to the red line to angle the block
        moveBackward(1.1, 0.4);//moving the block backward into zone
        turnLeftToHeading(20, 0.4);//turning to 0 degrees
        strafeRight(1.4, 0.4);//strafing before going forward
        moveForward(2.3, 0.4);//moving to 2nd block
        strafeLeft(1.2, 0.4);//strafing left to 2nd block
        moveBackward(2, 0.2);//moving to red zone
        turnRightToHeading(20, 0.4);//turning to face the red zone
        moveBackward(1.5, 0.2);//placing block in red zone


        // End of autonomous program
        telemetry.addData("Path", "Complete");
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.update();
        sleep(5000);
    }
    private void acceleration(double secondsToDrive, double speedToDrive,
                              double leftFrontDriveDirection, double rightFrontDriveDirection,
                              double leftBackDriveDirection, double rightBackDriveDirection){
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

            leftFrontDrive.setPower(currentSpeed*leftFrontDriveDirection);
            rightFrontDrive.setPower(currentSpeed*rightFrontDriveDirection);
            leftBackDrive.setPower(currentSpeed*leftBackDriveDirection);
            rightBackDrive.setPower(currentSpeed*rightBackDriveDirection);

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
        acceleration(secondsToDrive, speedToDrive, 1, 1, 1, 1);
    }

    private void moveBackward(double secondsToDrive) {
        moveBackward(secondsToDrive, DEFAULT_SPEED);
    }

    private void moveBackward(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, -1,-1,-1);
    }

    private void turnLeft(double secondsToDrive) {
        turnLeft(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnLeft(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, 1, -1, 1);
    }

    private void turnRight(double secondsToDrive) {
        turnRight(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnRight(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, -1, 1, -1);
    }

    private void strafeLeft(double secondsToDrive) {
        strafeLeft(secondsToDrive, DEFAULT_SPEED);
    }

    private void strafeRight(double secondsToDrive) {
        strafeRight(secondsToDrive, DEFAULT_SPEED);
    }

    private void strafeLeft(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, 1, 1, -1);
    }
    private void strafeRight(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, -1, -1, 1);
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

//NC TELEMETRY TRANSFER