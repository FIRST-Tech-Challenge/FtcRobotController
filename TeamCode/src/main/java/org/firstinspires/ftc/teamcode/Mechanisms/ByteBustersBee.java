package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ByteBustersBee
{
    //Define all the motors
    private DcMotor frontRightMotor, frontLeftMotor, backLeftMotor, backRightMotor;
    private double ticksPerRev; //revolution
    //Define all Servos

     public void init(HardwareMap hwMap)
     {    //************ DC Motors *****************//
        frontRightMotor = hwMap.get(DcMotor.class, "motor_fr");// "Motor FR" needs to match exactly as in configuration
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = frontRightMotor.getMotorType().getTicksPerRev();
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor frontLeftMotor = hwMap.get(DcMotor.class, "motor_fl");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         DcMotor backLeftMotor = hwMap.get(DcMotor.class, "motor_bl");
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         DcMotor backRightMotor = hwMap.get(DcMotor.class, "motor_br");
         backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //********************DC Motor initialization and methods ********************
    public void setFrontRightMotorSpeed(double speed)
    {
        //accepts values from -1.0 to 1.0
        frontRightMotor.setPower(speed);
    }
    public void setFrontLeftMotorSpeed(double speed)
    {
        //accepts values from -1.0 to 1.0
        frontLeftMotor.setPower(speed);
    }
    public void setBackLeftMotorSpeed(double speed)
    {
        //accepts values from -1.0 to 1.0
        backLeftMotor.setPower(speed);
    }
    public void setBackRightMotorSpeed(double speed)
    {
        //accepts values from -1.0 to 1.0
        backRightMotor.setPower(speed);
    }
    public double getFrontMotorRevs()
    {
                //normalizing ticks to revolutions
                return frontRightMotor.getCurrentPosition()/ticksPerRev;
    }
/*
1.add a method on ByteBusterBee that allows to change the break behaviour from opmode. On A press set break
 on B press set float */
    public void setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior zeroBehaviour)
    {
        frontRightMotor.setZeroPowerBehavior(zeroBehaviour);
    }
}
