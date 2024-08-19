package org.firstinspires.ftc.teamcode.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {


    double impulseRotation;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation){
        super(FLM, FRM, BLM, BRM);

        this.impulseRotation = impulseRotation;

        driveMotors[0].setDirection(DcMotorSimple.Direction.FORWARD); //FLM
        driveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE); //BLM
        driveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE); //BRM
        driveMotors[3].setDirection(DcMotorSimple.Direction.REVERSE); //FRM

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


        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1


        driveMotors[0].setPower((rotY + rotX + turn) / denominator); // Front Left Motor
        driveMotors[1].setPower((rotY - rotX + turn) / denominator); // Back Left Motor
        driveMotors[2].setPower((rotY - rotX - turn) / denominator); // Back Right Motor
        driveMotors[3].setPower((rotY + rotX - turn) / denominator); // Front Right Motor

        telemetry.addData("heading_DEGREES", Math.toDegrees(heading_RADIANS));
        telemetry.addData("heading_RADIANS", heading_RADIANS);
        telemetry.addData("drive", leftY);
        telemetry.addData("strafe", leftX);
        telemetry.addData("turn", turn);
        telemetry.addData("denominator", denominator);
        telemetry.update();




    }
    public double getAvgPos(){
       return driveMotors[0].getCurrentPosition()
               + driveMotors[1].getCurrentPosition()
               + driveMotors[2].getCurrentPosition()
               + driveMotors[3].getCurrentPosition();
    }

}
