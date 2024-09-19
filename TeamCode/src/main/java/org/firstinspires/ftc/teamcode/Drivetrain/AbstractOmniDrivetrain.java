package org.firstinspires.ftc.teamcode.Drivetrain;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;

import java.util.Map;

public abstract class AbstractOmniDrivetrain {

    double impulseRotation;
    RobotClass robotClass;
    Map<RobotClass.MOTORS, DcMotor> driveMotors;

    public AbstractOmniDrivetrain(Map<RobotClass.MOTORS, DcMotor> driveMotors, double impulseRotation, RobotClass robotClass) {

        this.robotClass = robotClass;
        this.impulseRotation = impulseRotation;
        this.driveMotors = driveMotors;
        driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.FORWARD); //FLM
        driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE); //BLM
        driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.REVERSE); //BRM
        driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD); //FRM

    }


    @SuppressLint("DefaultLocale")
    public void mecanumDrive(double leftY, double leftX, double turn, double heading_RADIANS, Telemetry telemetry) {
        //heading_DEGREES -= Math.toDegrees(Math.PI / 2);

        // drive == y strafe == x

        //starting value off by 90 degrees; 270 == -90
        //heading_RADIANS += Math.toRadians(90);


        double rotX = leftX * Math.cos(-heading_RADIANS) - leftY * Math.sin(-heading_RADIANS);

        double rotY = leftX * Math.sin(-heading_RADIANS) + leftY * Math.cos(-heading_RADIANS);


        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1

        driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower((rotY + rotX + turn) / denominator);
        driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower((rotY - rotX + turn) / denominator);
        driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower((rotY - rotX - turn) / denominator);
        driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower((rotY + rotX - turn) / denominator);

        telemetry.addData("heading_DEGREES", Math.toDegrees(heading_RADIANS));
        telemetry.addData("drive", leftY);
        telemetry.addData("strafe", leftX);
        telemetry.addData("turn", turn);
        telemetry.addData("denominator", denominator);
        telemetry.addData("RotationRate", robotClass.getRotationRate());
        //  telemetry.addLine(String.format("wheelSpeeds %6.1f %6.1f %6.1f %6.1f (speed)",  correctedWheelDrift[0], correctedWheelDrift[1], correctedWheelDrift[2], correctedWheelDrift[3]));
        telemetry.update();
    }

}