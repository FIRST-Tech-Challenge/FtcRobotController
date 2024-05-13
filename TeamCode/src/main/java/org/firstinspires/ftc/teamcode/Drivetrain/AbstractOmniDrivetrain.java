package org.firstinspires.ftc.teamcode.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {


    double impulseRotation;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation){
        super(FLM, FRM, BLM, BRM);

        this.impulseRotation = impulseRotation;

        driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setPowerBasic(double power){
        for(DcMotor motor : driveMotors){
            motor.setPower(power);
        }
    }
    public void mecanumDrive(double leftY, double leftX, double turn, double heading_RADIANS, Telemetry telemetry){
        //heading_DEGREES -= Math.toDegrees(Math.PI / 2);

        // drive == y strafe == x

        //starting value off by 90 degrees; 270 == -90
        heading_RADIANS += Math.toRadians(90);


        double rotY = leftX * Math.cos(-heading_RADIANS) - leftY * Math.sin(-heading_RADIANS);

        //IF robot strafe or forward movement is inverted after turn 90 degrees inverse either the rotX or rotY
        double rotX = leftX * Math.sin(heading_RADIANS) - leftY * Math.cos(heading_RADIANS);


        double denominator = Math.max(Math.abs(leftX) + Math.abs(leftY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1


        driveMotors[2].setPower((rotY + rotX + turn) / denominator); //front_left
        driveMotors[0].setPower((rotY - rotX + turn) / denominator); //back_left
        driveMotors[3].setPower((rotY - rotX - turn) / denominator); //front_right
        driveMotors[1].setPower((rotY + rotX - turn) / denominator); //back_right

        telemetry.addData("heading_DEGREES", Math.toDegrees(heading_RADIANS));
        telemetry.addData("heading_RADIANS", heading_RADIANS);
        telemetry.addData("drive", leftY);
        telemetry.addData("strafe", leftX);
        telemetry.addData("turn", turn);
        telemetry.update();




    }

}
