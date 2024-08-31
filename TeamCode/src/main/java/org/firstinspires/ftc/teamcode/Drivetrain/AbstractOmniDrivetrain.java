package org.firstinspires.ftc.teamcode.Drivetrain;
import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {

    Drive drive;
    double impulseRotation;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation, HardwareMap hwmap){
        super(FLM, FRM, BLM, BRM);
        drive = new Drive(hwmap);

        this.impulseRotation = impulseRotation;

        driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD); //FLM
        driveMotors[1].setDirection(DcMotorSimple.Direction.FORWARD); //BLM
        driveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE); //BRM
        driveMotors[3].setDirection(DcMotorSimple.Direction.FORWARD); //FRM

    }

    public void setPowerBasic(double power){
        for(DcMotor motor : driveMotors){
            motor.setPower(power);
        }
    }
    @SuppressLint("DefaultLocale")
    public void mecanumDrive(double leftY, double leftX, double turn, double heading_RADIANS, Telemetry telemetry){
        //heading_DEGREES -= Math.toDegrees(Math.PI / 2);

        // drive == y strafe == x

        //starting value off by 90 degrees; 270 == -90
        //heading_RADIANS += Math.toRadians(90);


        double rotY = leftX * Math.cos(heading_RADIANS) - leftY * Math.sin(heading_RADIANS);

        //IF robot strafe or forward movement is inverted after turn 90 degrees inverse either the rotX or rotY
        double rotX = leftX * Math.sin(heading_RADIANS) - leftY * Math.cos(heading_RADIANS);


        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1

        drive.DriveCartesian(rotX, rotY, turn);
        double[] wheelSpeeds = drive.getWheelSpeeds();
        double[] correctedWheelDrift;
        if(turn == 0){
            correctedWheelDrift = drive.correctDrift(wheelSpeeds, telemetry);
        }
        else{
            correctedWheelDrift = wheelSpeeds;
        }

        setPower(correctedWheelDrift);

        telemetry.addData("heading_DEGREES", Math.toDegrees(heading_RADIANS));
        telemetry.addData("drive", leftY);
        telemetry.addData("strafe", leftX);
        telemetry.addData("turn", turn);
        telemetry.addData("denominator", denominator);
        telemetry.addLine(String.format("wheelSpeeds %6.1f %6.1f %6.1f %6.1f (speed)",  correctedWheelDrift[0], correctedWheelDrift[1], correctedWheelDrift[2], correctedWheelDrift[3]));
        telemetry.update();
    }
    private void setPower(double[] wheelSpeeds){
        drive.drivetrain.driveMotors[RobotClass.kFrontLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        drive.drivetrain.driveMotors[RobotClass.kFrontRight].setPower(wheelSpeeds[RobotClass.kFrontRight]);
        drive.drivetrain.driveMotors[RobotClass.kBackLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        drive.drivetrain.driveMotors[RobotClass.kBackRight].setPower(wheelSpeeds[RobotClass.kBackRight]);
    }
}