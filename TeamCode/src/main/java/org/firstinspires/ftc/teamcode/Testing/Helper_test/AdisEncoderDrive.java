package org.firstinspires.ftc.teamcode.Testing.Helper_test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//This program was made by Adi. It is meant to be where we can acess documentation about
@Autonomous(name="AdisEncoderDrive", group = "tool")

//@Disabled
public class AdisEncoderDrive extends LinearOpMode
{
    // Adjust these numbers to suit your robot.

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_MM   = 96.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotorImpl lf_drive;  //  Used to control the left front drive wheel
    private DcMotorImpl rf_drive;  //  Used to control the right front drive wheel
    private DcMotorImpl lb_drive;  //  Used to control the left back drive wheel
    private DcMotorImpl rb_drive;//  Used to control the right back drive wheel


    //private DcMotor arm = null; // Used to control arm
   // Servo grip; // Used to control gripper


    @Override
    public void runOpMode()
    {
    }
    public void prepareEncoder(){

//        lf_drive = hardwareMap.get(DcMotorImpl.getClass(), "lf_drive");
//        rf_drive = hardwareMap.get(DcMotorImpl.getClass(), "rf_drive");
//        lb_drive = hardwareMap.get(DcMotorImpl.getClass(), "lb_drive");
//        rb_drive = hardwareMap.get(DcMotorImpl.getClass(), "rb_drive");

        lf_drive.setDirection(DcMotorImpl.Direction.REVERSE);
        lb_drive.setDirection(DcMotorImpl.Direction.REVERSE);

        //  arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_USING_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_USING_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_USING_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_USING_ENCODER);

    }

    public void encoderDriveForward(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition((int)TotalTicks);
        lb_drive.setTargetPosition((int)TotalTicks);
        rf_drive.setTargetPosition((int)TotalTicks);
        rb_drive.setTargetPosition((int)TotalTicks);
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void encoderDriveBackward(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRight(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeft(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(((int)TotalTicks));
        rf_drive.setTargetPosition(((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveForwardBlocks(double blocks){
        double TotalTicks = blocks*COUNTS_PER_MM*609.6;
        lf_drive.setTargetPosition((int)TotalTicks);
        lb_drive.setTargetPosition((int)TotalTicks);
        rf_drive.setTargetPosition((int)TotalTicks);
        rb_drive.setTargetPosition((int)TotalTicks);
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(blocks*609.6*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void encoderDriveBackwardBlocks(double Blocks){
        double TotalTicks = Blocks*COUNTS_PER_MM*609.6;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(Blocks*609.6*300));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRightBlocks(double blocks){
        double TotalTicks = blocks*COUNTS_PER_MM*609.6;
        lf_drive.setTargetPosition(((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(blocks*2.5*609.6));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeftBlocks(double blocks){
        double TotalTicks = blocks*COUNTS_PER_MM*609.6;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(((int)TotalTicks));
        rf_drive.setTargetPosition(((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(blocks*2.5*609.6));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveForwardInches(double inches){
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lf_drive.setTargetPosition((int)TotalTicks);
        lb_drive.setTargetPosition((int)TotalTicks);
        rf_drive.setTargetPosition((int)TotalTicks);
        rb_drive.setTargetPosition((int)TotalTicks);
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2.5));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void encoderDriveBackwardInches(double Inches){
        double TotalTicks = Inches*COUNTS_PER_MM*25.4;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(Inches*2.5*25.4));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRightInches(double inches){
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lf_drive.setTargetPosition(((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*2.5*25.4));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeftInches(double inches){
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(((int)TotalTicks));
        rf_drive.setTargetPosition(((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*2.5*25.4));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void TurnLeft(double degrees){
        double TotalTicks = (degrees/90)*COUNTS_PER_MOTOR_REV*4.2333333333;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(((int)TotalTicks));
        rb_drive.setTargetPosition(((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(degrees/90*960));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void TurnRight(double degrees){
        double TotalTicks = (degrees/90)*COUNTS_PER_MOTOR_REV*4.2333333333;
        lf_drive.setTargetPosition(((int)TotalTicks));
        lb_drive.setTargetPosition(((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotorImpl.RunMode.RUN_TO_POSITION);
        sleep((long)(degrees/90*960));
        lf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotorImpl.RunMode.STOP_AND_RESET_ENCODER);
    }



}