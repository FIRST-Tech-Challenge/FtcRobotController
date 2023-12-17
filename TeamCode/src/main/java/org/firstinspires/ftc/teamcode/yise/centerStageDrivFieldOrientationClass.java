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


public class centerStageDrivFieldOrientationClass {
    // Note: we make these public so the calling code can access and use these variables
    public final DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private List<DcMotor> motors;
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    public double pi = Math.PI;

    public double piOverTwo = Math.PI/2;

    private double AngleA = pi;
    //private double DeadzoneA = 0.12; original value of dead zone
    private double DeadzoneA = .06;

    private IMU imu;


    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;

    public enum Speeds {
        SLOW,
        NORMAL
    }


    public centerStageDrivFieldOrientationClass(HardwareMap hardwareMap) {
        // set default value for speed
        currentSpeed = Speeds.NORMAL;

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

        // Set default power for motors
        leftFrontPower = 1;
        leftBackPower = 1;
        rightFrontPower = 1;
        rightBackPower = 1;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    // Updates power to the 4 drive motors based on input from the stick on the first controller
    public void updateMotorsFromStick(Gamepad gamepad) {
        float speedMultiplier;
        double vertical, horizontal, turn;
        double verticalOut, horizontalOut;



        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double theta = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;

        double max;



        // Set the default value of speedMultiplier for NORMAL mode
        speedMultiplier = 1f;


        // Set the speedMultiplier in case of SLOW mode
        if (currentSpeed == Speeds.SLOW) {
            speedMultiplier = 0.5f;
        }

        // Assign human readable names to the stick inputs
        vertical = gamepad.left_stick_y;
        horizontal = gamepad.left_stick_x;
        turn = gamepad.right_stick_x;



        //field orientation math switching motor inputs to field oriented outputs
        horizontalOut = (horizontal * Math.cos(theta)) - (vertical * Math.sin(theta));
        verticalOut = (vertical * Math.cos(theta)) + (horizontal * Math.sin(theta));

        //note: vertical input is negative so we flip the output positive
        verticalOut = verticalOut * -1;

        leftFrontPower  = (verticalOut - horizontalOut - turn);
        rightFrontPower = (verticalOut + horizontalOut + turn);
        leftBackPower   = (verticalOut + horizontalOut - turn);
        rightBackPower  = (verticalOut - horizontalOut + turn);

        //Implementation of having power to the motors to brake when not moving
        if (vertical == 0 && horizontal == 0 && turn == 0){
            leftFrontPower  = 0.01;
            rightFrontPower = 0.01;
            leftBackPower   = 0.01;
            rightBackPower  = 0.01;
        }

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

    // Updates power to the 4 drive motors based on input from the stick on the first controller
    public void updateMotorsFromDpad(Gamepad gamepad) {
        double power;

        // Set default value for power
        power = .5f;

        // Set the power to send to the motors based on currentSpeed setting
        if (currentSpeed == Speeds.SLOW) {
            power = 0.25f;
        }

        if (gamepad.dpad_right) {
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power * -1);
            rightFrontDrive.setPower(power * -1);
        } else if (gamepad.dpad_left) {
            leftFrontDrive.setPower(power * -1);
            rightBackDrive.setPower(power * -1);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        } else if (gamepad.dpad_down) {
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        } else if (gamepad.dpad_up) {
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
        }
    }

    public void lockToRotation(Gamepad gamepad) {
        double turnR, turnL;


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double theta = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;

        turnL = 0;
        turnR = 0;

        if (gamepad.b && theta > AngleA + DeadzoneA) {
            turnR = (((theta - AngleA) - DeadzoneA) / -piOverTwo);
        }else if (gamepad.b && theta < AngleA - DeadzoneA) {
            //CURRENT VERSION of the auto lock code
            turnL = (((theta - AngleA) - DeadzoneA) / piOverTwo);
        } else if (!gamepad.dpad_down && !gamepad.b && !gamepad.dpad_right && !gamepad.dpad_left) {
            turnR = 0;
            turnL = gamepad.right_stick_x;
        } else if (gamepad.b && theta < 3.2 && theta > 3.08){
            turnL = 0;
            turnR = 0;
        }
        leftFrontPower  = (turnR - turnL);
        rightFrontPower = (-turnR + turnL);
        leftBackPower   = (turnR - turnL);
        rightBackPower  = (-turnR + turnL);
    }

        // Changes the drive speed mode to slow
    public void setSlowMode() {
        currentSpeed = Speeds.SLOW;
    }

    // Change the drive speed mode to normal
    public void setNormalMode() {
        currentSpeed = Speeds.NORMAL;
    }
}

