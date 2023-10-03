package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
        imu.resetYaw();
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
    public void waitStopped(int time){
        stopM();
        lom.sleep(time);
    }
    public void rotateFeildCentric(double power, int degrees){
        double TargetAngle = degrees;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double deltaDegrees = (TargetAngle - botHeading + 540) % 360 - 180;
        double yaw = 0;
        double kp = -0.033;
        while (true){

            if ((Math.abs(deltaDegrees) < 1.0 && Math.abs(power) < 2.0)){
                break;
            }
            else {
                yaw = kp * deltaDegrees / power;
            }
            double leftFrontPower  = yaw;
            double rightFrontPower = -yaw;
            double leftBackPower   = yaw;
            double rightBackPower  = -yaw;
            leftFrontDrive.setPower(leftFrontPower * power);
            rightFrontDrive.setPower(rightFrontPower * power);
            leftBackDrive.setPower(leftBackPower * power);
            rightBackDrive.setPower(rightBackPower * power);

        }
    }
    public void rotateDegrees(double power, int degrees){
        double TargetAngle = degrees;
        int stop = 1000;
        while (stop > 0){
            stop -= 1;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double deltaDegrees = (TargetAngle - botHeading + 540) % 360 - 180;
            double yaw = 0;
            double kp = -0.033;
            if ((Math.abs(deltaDegrees) < 1.0 && Math.abs(power) < 2.0)){
                break;
            }
            else {
                yaw = kp * deltaDegrees / power;
            }
            double leftFrontPower  = yaw;
            double rightFrontPower = -yaw;
            double leftBackPower   = yaw;
            double rightBackPower  = -yaw;
            leftFrontDrive.setPower(leftFrontPower * power);
            rightFrontDrive.setPower(rightFrontPower * power);
            leftBackDrive.setPower(leftBackPower * power);
            rightBackDrive.setPower(rightBackPower * power);

        }
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
