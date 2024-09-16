package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.Drive;

public abstract class AbstractDrivetrain {
    public DcMotor[] driveMotors;
    public AbstractDrivetrain(DcMotor FLM, DcMotor BLM, DcMotor FRM, DcMotor BRM){

        //FLM == "Front Left Motor" etc.

        driveMotors = new DcMotor[]{FLM, BLM, BRM, FRM};
        driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveBasic(double leftY, double rightX) {
       double[] wheelSpeeds =  Drive.DriveSixWheel(leftY, rightX);
       setPower(wheelSpeeds);
    }
    public void driveTank(double leftY, double rightY){
        driveMotors[0].setPower(rightY);
        driveMotors[1].setPower(leftY);
        driveMotors[2].setPower(rightY);
        driveMotors[3].setPower(leftY);
    }

    public void setMode(DcMotor.RunMode mode) {
        for(DcMotor motor : driveMotors) {
            motor.setMode(mode);
        }
    }
    private void setPower(double[] wheelSpeeds){
        driveMotors[RobotClass.kFrontLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        driveMotors[RobotClass.kFrontRight].setPower(wheelSpeeds[RobotClass.kFrontRight]);
        driveMotors[RobotClass.kBackLeft].setPower(wheelSpeeds[RobotClass.kFrontLeft]);
        driveMotors[RobotClass.kBackRight].setPower(wheelSpeeds[RobotClass.kBackRight]);
    }
}
