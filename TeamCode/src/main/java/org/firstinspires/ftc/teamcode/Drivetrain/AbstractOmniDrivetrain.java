package org.firstinspires.ftc.teamcode.Drivetrain;
import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {

    double impulseRotation;
    RobotClass robotClass;
    int frontLeft = RobotClass.kFrontLeft;
    int frontRight = RobotClass.kFrontRight;
    int backLeft = RobotClass.kBackLeft;
    int backRight = RobotClass.kBackRight;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor BLM, DcMotor FRM, DcMotor BRM, double impulseRotation, RobotClass robotClass){
        super(FLM, BLM, FRM, BRM);
        this.robotClass = robotClass;
        this.impulseRotation = impulseRotation;

        driveMotors[frontLeft].setDirection(DcMotorSimple.Direction.FORWARD); //FLM
        driveMotors[backLeft].setDirection(DcMotorSimple.Direction.REVERSE); //BLM
        driveMotors[backRight].setDirection(DcMotorSimple.Direction.REVERSE); //BRM
        driveMotors[frontRight].setDirection(DcMotorSimple.Direction.FORWARD); //FRM

    }


    @SuppressLint("DefaultLocale")
    public void mecanumDrive(double leftY, double leftX, double turn, double heading_RADIANS, Telemetry telemetry){
        //heading_DEGREES -= Math.toDegrees(Math.PI / 2);

        // drive == y strafe == x

        //starting value off by 90 degrees; 270 == -90
        //heading_RADIANS += Math.toRadians(90);


        double rotX = leftX * Math.cos(-heading_RADIANS) - leftY * Math.sin(-heading_RADIANS);

        double rotY = leftX * Math.sin(-heading_RADIANS) + leftY * Math.cos(-heading_RADIANS);


        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1

        driveMotors[RobotClass.kFrontLeft].setPower((rotY + rotX + turn) / denominator);
        driveMotors[RobotClass.kBackLeft].setPower((rotY - rotX + turn) / denominator);
        driveMotors[RobotClass.kFrontRight].setPower((rotY - rotX - turn) / denominator);
        driveMotors[RobotClass.kBackRight].setPower((rotY + rotX - turn) / denominator);

        telemetry.addData("heading_DEGREES", Math.toDegrees(heading_RADIANS));
        telemetry.addData("drive", leftY);
        telemetry.addData("strafe", leftX);
        telemetry.addData("turn", turn);
        telemetry.addData("denominator", denominator);
        telemetry.addData("RotationRate", robotClass.getRotationRate());
      //  telemetry.addLine(String.format("wheelSpeeds %6.1f %6.1f %6.1f %6.1f (speed)",  correctedWheelDrift[0], correctedWheelDrift[1], correctedWheelDrift[2], correctedWheelDrift[3]));
        telemetry.update();
    }
    private void setPower(double[] wheelSpeeds){
       driveMotors[RobotClass.kFrontLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
       driveMotors[RobotClass.kFrontRight].setPower(wheelSpeeds[RobotClass.kFrontRight]);
       driveMotors[RobotClass.kBackLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
       driveMotors[RobotClass.kBackRight].setPower(wheelSpeeds[RobotClass.kBackRight]);
    }
}