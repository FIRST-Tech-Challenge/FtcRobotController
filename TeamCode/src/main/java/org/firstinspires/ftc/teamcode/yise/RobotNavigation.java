package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;

public class RobotNavigation {
    // Note: we make these public so the calling code can access and use these variables
    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    public IMU imu;
    private List<DcMotor> motors;
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    double speedMultiplier;
    double vertical, horizontal, turn;
    double max;


    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;

    public enum Speeds {
        SLOW,
        NORMAL
    }

    //Declare the constructor for the class
    public RobotNavigation(HardwareMap hardwareMap) {
        // set default value for speed
        currentSpeed = Speeds.NORMAL;
        speedMultiplier = 1;

        // Set the drive motor variables based on hardware config
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive);

        // Set the direction of the motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        //Field orientation stuff
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }


    public void updateMotorsFromStick(Gamepad gamepad) {

        //Get stick inputs
        vertical = gamepad.left_stick_y;
        horizontal = gamepad.left_stick_x;
        turn = gamepad.right_stick_x;


        // Calculate individual motor power base on the stick input values
        leftFrontPower = vertical - horizontal - turn;
        rightFrontPower = vertical + horizontal + turn;
        leftBackPower = vertical + horizontal - turn;
        rightBackPower = vertical - horizontal + turn;


        // Normalize the power values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Set the power on the motors
        leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
        rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
        leftBackDrive.setPower(leftBackPower * speedMultiplier);
        rightBackDrive.setPower(rightBackPower * speedMultiplier);
    }


    public void updateMotorsFieldOrientation(Gamepad gamepad) {
        double vertical, horizontal, turn;
        double verticalOut, horizontalOut;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double theta = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;

        double max;


        // Assign human readable names to the stick inputs
        vertical = gamepad.left_stick_y;
        horizontal = gamepad.left_stick_x;
        turn = gamepad.right_stick_x;


        //field orientation math switching motor inputs to field oriented outputs
        //note: vertical input is negative so we flip the output positive
        horizontalOut = (horizontal * Math.cos(theta)) - (vertical * Math.sin(theta));
        verticalOut = -(vertical * Math.cos(theta)) + (horizontal * Math.sin(theta));

        leftFrontPower  = (verticalOut + horizontalOut - turn);
        rightFrontPower = (verticalOut - horizontalOut + turn);
        leftBackPower   = (verticalOut - horizontalOut - turn);
        rightBackPower  = (verticalOut + horizontalOut + turn);


        // Normalize the power values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Set the power on the motors
        leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
        rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
        leftBackDrive.setPower(leftBackPower * speedMultiplier);
        rightBackDrive.setPower(rightBackPower * speedMultiplier);
    }


    public void updateMotorsFromDpad(Gamepad gamepad) {

        //Set output values according to what was pressed
        if (gamepad.dpad_right) {
            horizontal = 1;
        } else if (gamepad.dpad_left) {
            horizontal = -1;
        } else if (gamepad.dpad_down) {
            vertical = -1;
        } else if (gamepad.dpad_up) {
            vertical = 1;
        }

        leftFrontPower = vertical + horizontal - turn;
        rightFrontPower = vertical - horizontal + turn;
        leftBackPower = vertical - horizontal - turn;
        rightBackPower = vertical + horizontal + turn;

        leftFrontDrive.setPower(leftFrontPower);
        rightBackDrive.setPower(rightBackPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
    }

    //Toggles speed
    public void toggleSlowMode(Speeds targetSpeed) {

        // Set the speedMultiplier in case of SLOW mode
        if (currentSpeed == Speeds.SLOW) {
            currentSpeed = Speeds.NORMAL;
            speedMultiplier = 1;
        } else {
            currentSpeed = Speeds.SLOW;
            speedMultiplier = 0.5;
        }
    }
}

