package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by WilliamsburgRobotic on 10/31/2017.
 */

public class MoveHelper {


    private static final double ENCODER_POWER_LEVEL = 1;
    // declares the motors; gives them names we will use to call them later
    protected DcMotor FLMotor;
    protected DcMotor FRMotor;
    protected DcMotor BLMotor;
    protected DcMotor BRMotor;
    private boolean isPositionValid;
    public double encoderPowerLevel = 1;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    MoveHelper(Telemetry t, HardwareMap h) {
        hardwareMap = h;
        telemetry = t;
    }

    public void init() {
        // links motor names here to the names given in the config on the phones
        FLMotor = hardwareMap.dcMotor.get("LF"); // TODO: Fix the config names to match the variables
        FRMotor = hardwareMap.dcMotor.get("RF");
        BLMotor = hardwareMap.dcMotor.get("LB");
        BRMotor = hardwareMap.dcMotor.get("RB");


        // setting directions/telling them we are using encoders
        //if (isOldRobot) {
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        /*} else {
            BRMotor.setDirection(DcMotor.Direction.REVERSE);
            FLMotor.setDirection(DcMotor.Direction.REVERSE);
        }*/
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void omniDrive(double lx,double ly, double rx){
        telemetry.addData("Drive input (lx,ly): ", lx + "," + ly);
        // omni-drive math, sets it up to run properly
        double fl = ly - lx - rx;
        double fr = ly + lx + rx;
        double bl = ly + lx - rx;
        double br = ly - lx + rx;


        String output = String.format("%1$.3f,%1$.3f,%1$.3f,%1$.3f",fl,fr,bl,br);
        telemetry.addData("Driving (fl,fr,bl,br): ", output);
        // sets power to the motors
        FLMotor.setPower(fl);
        FRMotor.setPower(fr);
        BLMotor.setPower(bl);
        BRMotor.setPower(br);

    }

    public void turn(double rx){
        // method used to turn the robot
        FLMotor.setPower(rx);
        FRMotor.setPower(-rx);
        BLMotor.setPower(rx);
        BRMotor.setPower(-rx);
    }


    // actually turns on the motors/sets power??
    public void runFLMotor (double power){
        FLMotor.setPower(power);
    }
    public void runFRMotor (double power){
        FRMotor.setPower(power);
    }
    public void runBRMotor (double power){
        BRMotor.setPower(power);
    }

    public void driveForward (double power){
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
    }



    public void loop(){
    }
    public void showEncoderValues (){
        telemetry.addData("BR Encoder", BRMotor.getCurrentPosition());
        telemetry.addData("BL Encoder", BLMotor.getCurrentPosition());
        telemetry.addData("FR Encoder", FRMotor.getCurrentPosition());
        telemetry.addData("FL Encoder", FLMotor.getCurrentPosition());

    }
    public void resetEncoders (){
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        isPositionValid = false;
    }
    public void runUsingEncoders (){
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders(){
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveToPosition (int position){
        FLMotor.setTargetPosition(position);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getEncoderValue(){
        return FLMotor.getCurrentPosition();

    }

    public void runOneMotor(DcMotor motor, int position){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(encoderPowerLevel);
    }

    public void continueOneMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(encoderPowerLevel);
        telemetry.addData("Continue target: " + motor.getDeviceName(),motor.getTargetPosition());
    }

    public void runMotorsToPosition(int flPos, int frPos, int blPos, int brPos){
        if (!isPositionValid) {
            runOneMotor(FLMotor, flPos);
            runOneMotor(FRMotor, frPos);
            runOneMotor(BRMotor, brPos);
            runOneMotor(BLMotor, blPos);
            isPositionValid = true;
        }
    }

    public void continueToPosition(){
        if (isPositionValid) {
            continueOneMotor(FLMotor);
            continueOneMotor(FRMotor);
            continueOneMotor(BRMotor);
            continueOneMotor(BLMotor);
        }
    }

    public void checkTeleOp(Gamepad gamepad1,Gamepad gamepad2){
        // alaina is struggling to find a way to describe this
        double LY = -gamepad1.left_stick_y;
        double LX = gamepad1.left_stick_x;
        double RX = -gamepad1.right_stick_x;

     /*   if (gamepad1.y) {
            LY = 0.5;
        }
        if (gamepad1.a) {
            LY = -.5;
        }
        if (gamepad1.x) {
            RX = -.5;
        }
        if (gamepad1.b) {
            RX = .5;
        }*/


        //Establishes floating variables linked to the gamepads
        telemetry.addData("Left X", LX);
        telemetry.addData("Left Y", LY);
        telemetry.addData("Right X", RX);
        telemetry.addData("BR Encoder", BRMotor.getCurrentPosition());
        telemetry.addData("BL Encoder", BLMotor.getCurrentPosition());
        telemetry.addData("FR Encoder", FRMotor.getCurrentPosition());
        telemetry.addData("FL Encoder", FLMotor.getCurrentPosition());
        LY = Range.clip(LY, -1, 1);
        LX = Range.clip(LX, -1, 1);
        RX = Range.clip(RX, -1, 1);

        omniDrive(LX, LY, RX*.75);

        if (gamepad1.a) {
            BLMotor.setPower(.3);
        }
        if (gamepad1.b) {
            BRMotor.setPower(.3);
        }
        if (gamepad1.x) {
            FLMotor.setPower(.3);
        }
        if (gamepad1.y) {
            FRMotor.setPower(.3);
        }


    }
    public int GetBRMotorPosition(){
        return BRMotor.getCurrentPosition();
    }
    public int GetBLMotorPosition(){
        return BLMotor.getCurrentPosition();
    }
    public int GetFRMotorPosition(){
        return FRMotor.getCurrentPosition();
    }
    public int GetFLMotorPosition(){
        return FLMotor.getCurrentPosition();
    }

}