package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class DrivingFunctions {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU imu = null;
    private LinearOpMode lom = null;

    public DrivingFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    private void Initialize()
    {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = lom.hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = lom.hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = lom.hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = lom.hardwareMap.get(DcMotor.class, "backright");

        imu = lom.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void stopM() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void driveForward(double power, int miliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        lom.sleep(miliseconds);
        stopM();
    }


    public void driveBackward(double power, int miliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        lom.sleep(miliseconds);
        stopM();
    }

    public void rotateRight(double power, int miliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        lom.sleep(miliseconds);
        stopM();
    }

    public void rotateLeft(double power, int miliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        lom.sleep(miliseconds);
        stopM();
    }

    public void strafeRight(double power, int miliseconds) {
        int direction = 1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        lom.sleep(miliseconds);
        stopM();
    }

    public void strafeLeft(double power, int miliseconds) {
        int direction = -1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        lom.sleep(miliseconds);
        stopM();
    }

    public void returnToLocation(double power, int miliseconds, int direction) {

        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        lom.sleep(miliseconds);
        stopM();

        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction * -1);
        lom.sleep(miliseconds);
        stopM();
    }

}
