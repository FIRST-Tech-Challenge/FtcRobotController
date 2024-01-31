package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import java.util.List;

public class DriveSubsystem extends SubsystemBase
{
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private boolean reset=false;
    private double offset;

    public DriveSubsystem(HardwareMap hardwareMap)
    {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double heading(){
        if(reset){
            return normalizeRadians(RobotContainer.getImuAbsoluteOrientation()-offset);
        }return RobotContainer.getImuAbsoluteOrientation();
    }
    public void resetIMU() {
       reset=true;
       offset=RobotContainer.getImuAbsoluteOrientation();
    }

    public void runRobotCentric(double y,double x,double rx, double topSpeed) {
        run(y,x,rx, 0, topSpeed);
    }

    public void runFeildCentric(double y,double x,double rx, double topSpeed) {
        run(y,x,rx, heading(), topSpeed);
    }

    private void run(double y,double x,double rx, double botHeading, double topSpeed)
    {
        x = x * 1.1; // Counteract imperfect strafing

        y=y*y*y;
        x=x*x*x;
        rx=rx*rx*rx;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double LeftFrontPower = (rotY + rotX + rx) / denominator;
        double LeftRearPower = (rotY - rotX + rx) / denominator;
        double RightFrontPower = (rotY - rotX - rx) / denominator;
        double RightRearPower = (rotY + rotX - rx) / denominator;

        LeftFrontPower = Range.scale(LeftFrontPower,-1,1,-topSpeed,topSpeed);
        LeftRearPower = Range.scale(LeftRearPower,-1,1,-topSpeed,topSpeed);
        RightFrontPower = Range.scale(RightFrontPower,-1,1,-topSpeed,topSpeed);
        RightRearPower = Range.scale(RightRearPower,-1,1,-topSpeed,topSpeed);

        leftFront.setPower(LeftFrontPower);
        leftRear.setPower(LeftRearPower);
        rightFront.setPower(RightFrontPower);
        rightRear.setPower(RightRearPower);
    }
}