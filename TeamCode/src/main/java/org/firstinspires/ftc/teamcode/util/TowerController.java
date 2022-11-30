package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TowerController {

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor screw;
    private DcMotor uBar;
    private Servo intake;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    public boolean raiseTower;
    public boolean intakePos = false;
    private int uBarLevel;
    private int screwLevel;
    static final double     COUNTS_PER_MOTOR    = 384.5;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION);


    public TowerController (HardwareMap hardwareMap)
    {

        //Setup motors
        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
//        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");
        screw = hardwareMap.get(DcMotor.class, "screw");
//        uBar = hardwareMap.get(DcMotor.class, "uBar");
//        intake = hardwareMap.get(Servo.class, "intake");
        screw.setDirection(DcMotor.Direction.FORWARD);
//        uBar.setDirection(DcMotor.Direction.FORWARD);


        //Setup sensors
//        DigitalChannel highSensor = hardwareMap.get(DigitalChannel.class, "highSensor");
//        DigitalChannel lowSensor = hardwareMap.get(DigitalChannel.class, "lowSensor");
//        highSensor.setMode(DigitalChannel.Mode.INPUT);
//        lowSensor.setMode(DigitalChannel.Mode.INPUT);
        //setup encoder
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

//    public void handleScrew() {
//        if(raiseTower){
//            if(highSensor.getState()){
//                screw.setPower(0);
//            }
//            else{
//                screw.setPower(1);
//            }
//        }
//        else{
//            if(lowSensor.getState()){
//                screw.setPower(0);
//            }
//            else{
//                screw.setPower(-1);
//            }
//        }
//    }

//
//    private void driveUBar(double uBarTarget, double speed, Telemetry telemetry) {
//        uBarLevel -= uBarTarget;
//        uBar.setTargetPosition(uBarLevel);
//        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        uBar.setPower(speed);
//        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();
//        while (uBar.isBusy() && uBar.getCurrentPosition() <= uBarTarget)
//        {
//            telemetry.addData("UBar ticks = ", "%d", uBar.getCurrentPosition());
//            telemetry.update();
//            uBar.setPower(speed);
//        }
//        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        uBar.setPower(0);
//        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

    private void driveScrewUp(double screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel += screwTarget;
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();

            if (highSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();
        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    private void driveScrewDown(double screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (highSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/*
        if (highSensor.isPressed())
        {
            screw.setDirection(DcMotor.Direction.FORWARD);
            screwTarget = 476;
            screwLevel -= screwTarget;
            while (screw.isBusy() && screw.getCurrentPosition() <= screwTarget)
            {
                screw.setTargetPosition(screwLevel);
                screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                screw.setPower(speed);
            }
            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            screw.setPower(0);
            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        screwLevel = 0;

*/

    }

//    public void handleIntake () {
//        if(intakePos){
//            intake.setPosition(0.5);
//            intake.getPosition();
//        }
//        else{
//            intake.setPosition(0);
//            intake.getPosition();
//        }
//    }
    public void handleGamepad(Gamepad gamepad, Telemetry telemetry)
    {

        //Screw
        if(gamepad.dpad_up) {
            driveScrewUp(500, 0.1, telemetry);
        }
        if(gamepad.dpad_down) {
            driveScrewDown(500, -0.1, telemetry);
        }


//        //U Bar
//        int num = 0;
//        if(gamepad.b)
//        {
//            gamepad.b = false;
//            driveUBar(	330.06875, 0.2, telemetry);
//            //60 degrees
//        }
//        if(gamepad.a)
//        {
//            gamepad.a = false;
//            driveUBar(	165.034375, 0.2, telemetry);
//            //30 degrees
//        }
//        if(gamepad.x)
//        {
//            gamepad.x = false;
//            driveUBar(	-330.06875, 0.2, telemetry);
//            //60 degrees
//        }
//        if(gamepad.y)
//        {
//            gamepad.y = false;
//            driveUBar(	-165.034375, 0.2, telemetry);
//            //30 degrees
//        }
//
//        //Intake
//        if(gamepad.right_bumper)
//        {
//            intakePos = true;
//        }
//        if(gamepad.left_bumper)
//        {
//            intakePos = false;
//        }
//        handleIntake();
    }

//        public boolean limitSwitches(boolean stop,Telemetry telemetry)
//        {
//
//            if (highSensor.isPressed())
//            {
//                telemetry.addData("Status", "Limit Switch is Pressed");
//                screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                driveScrewUp(-100, 0.2, telemetry);
//                stop = true;
//                return stop;
//            }
//            else
//            {
//                telemetry.addData("Status", "Limit Switch is not Pressed");
//                stop = false;
//            }
//            telemetry.update();
//
//            if (lowSensor.isPressed())
//            {
//                telemetry.addData("Status", "Limit Switch is Pressed");
//                screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                driveScrewDown(100, 0.2, telemetry);
//                stop = true;
//                return stop;
//            }
//            else
//            {
//                telemetry.addData("Status", "Limit Switch is not Pressed");
//                stop = false;
//            }
//            telemetry.update();
//            return stop;
//        }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT