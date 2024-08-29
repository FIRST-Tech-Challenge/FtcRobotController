package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AbstractDrivetrain {
    public DcMotor[] driveMotors;
    public AbstractDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM){

        //FLM == "Front Left Motor" etc.

        driveMotors = new DcMotor[]{FLM, BLM, BRM, FRM};
        driveMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveBasic(double leftY, double rightX) {
        driveMotors[0].setPower(leftY + rightX);
        driveMotors[1].setPower(leftY - rightX);
        driveMotors[2].setPower(leftY + rightX);
        driveMotors[3].setPower(leftY - rightX);
    }
    public void driveTank(double leftY, double rightY){
        driveMotors[0].setPower(leftY);
        driveMotors[1].setPower(rightY);
        driveMotors[2].setPower(rightY);
        driveMotors[3].setPower(leftY);
    }

    public void setMode(DcMotor.RunMode mode) {
        for(DcMotor motor : driveMotors) {
            motor.setMode(mode);
        }
    }
}
