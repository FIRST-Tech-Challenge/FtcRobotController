package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSub extends SubsystemBase{
    public final DcMotor rightFront;
    public final DcMotor leftFront;
    public final DcMotor rightRear;
    public final DcMotor leftRear;
    public final IMU imu;
    public DriveSub(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        rightFront=hardwareMap.get(DcMotor.class, "rightFront");
        rightRear=hardwareMap.get(DcMotor.class, "rightRear");
        leftFront=hardwareMap.get(DcMotor.class, "leftFront");
        leftRear=hardwareMap.get(DcMotor.class, "leftRear");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetIMU(){
        imu.resetYaw();
    }
    public void runRobotCentric(double y,double x,double rx,boolean slow_mode){

        y = -y; // Remember, this is reversed!
        x = x * 1.1; // Counteract imperfect strafing
        double topS=1;
        if(slow_mode)topS=0.5;

        y=y*y*y;
        x=x*x*x;
        rx=rx*rx*rx;

        double botHeading = 0;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        frontRightPower= Range.scale(frontRightPower,-1,1,-topS,topS);
        frontLeftPower= Range.scale(frontLeftPower,-1,1,-topS,topS);
        backRightPower= Range.scale(backRightPower,-1,1,-topS,topS);
        backLeftPower= Range.scale(backLeftPower,-1,1,-topS,topS);
        run(frontRightPower,backRightPower,frontLeftPower,backLeftPower);
    }
    public void runFeildCentric(double y,double x,double rx,boolean slow_mode){
        y = -y; // Remember, this is reversed!
        x = x * 1.1; // Counteract imperfect strafing

        double topS=1;
        if(slow_mode)topS=0.5;

        y=y*y*y;
        x=x*x*x;
        rx=rx*rx*rx;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        frontRightPower= Range.scale(frontRightPower,-1,1,-topS,topS);
        frontLeftPower= Range.scale(frontLeftPower,-1,1,-topS,topS);
        backRightPower= Range.scale(backRightPower,-1,1,-topS,topS);
        backLeftPower= Range.scale(backLeftPower,-1,1,-topS,topS);
        run(frontRightPower,backRightPower,frontLeftPower,backLeftPower);
    }

    private void run(double rightFrontPower,double rightRearPower,double leftFrontPower,double leftRearPower) {
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
    }
}
