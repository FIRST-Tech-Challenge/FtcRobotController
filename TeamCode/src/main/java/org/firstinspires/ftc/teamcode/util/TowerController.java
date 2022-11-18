package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TowerController {

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor screw;
    private DcMotor uBar;
    private Servo intake;
//    private DigitalChannel highSensor;
//    private DigitalChannel lowSensor;
    public boolean raiseTower;
    public boolean intakePos = false;
    private int uBarLevel;
    static final double     COUNTS_PER_MOTOR    = 384.5;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION);

    public TowerController (HardwareMap hardwareMap){

        //Setup motors
//        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(Servo.class, "intake");
//        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.FORWARD);


        //Setup sensors
//        DigitalChannel highSensor = hardwareMap.get(DigitalChannel.class, "highSensor");
//        DigitalChannel lowSensor = hardwareMap.get(DigitalChannel.class, "lowSensor");
//        highSensor.setMode(DigitalChannel.Mode.INPUT);
//        lowSensor.setMode(DigitalChannel.Mode.INPUT);

        //setup encoder
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    private void drive(double uBarTarget, double speed, Telemetry telemetry) {
        uBarLevel -= uBarTarget;
        uBar.setTargetPosition(uBarLevel);
        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uBar.setPower(speed);
        telemetry.addData("Ticks is = ", "%d", uBar.getCurrentPosition());
        telemetry.update();
        while (uBar.isBusy() && uBar.getCurrentPosition() <= uBarTarget)
        {
            telemetry.addData("Ticks is = ", "%d", uBar.getCurrentPosition());
            telemetry.update();
            uBar.setPower(speed);
        }
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stop()
    {
        uBar.setPower(0);
    }

//    private void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
//    {
//        leftFrontPos += leftFrontTarget;
//        rightFrontPos += rightFrontTarget;
//        leftBackPos += leftBackTarget;
//        rightBackPos += rightBackTarget;
//
//        leftFront.setTargetPosition(leftFrontPos);
//        rightFront.setTargetPosition(rightFrontPos);
//        leftBack.setTargetPosition(leftBackPos);
//        rightBack.setTargetPosition(rightBackPos);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftFront.setPower(speed);
//        rightFront.setPower(speed);
//        leftBack.setPower(speed);
//        rightBack.setPower(speed);
//
//        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())
//        {
//            idle();
//        }
//
//    }

    public void handleIntake () {
        if(intakePos){
            intake.setPosition(0.5);
        }
        else{
            intake.setPosition(0);
        }
    }
    public void handleGamepad(Gamepad gamepad, Telemetry telemetry) {

        //Screw
        /* if(gamepad.dpad_up) {
            raiseTower = true;
        }
        if(gamepad.dpad_down) {
            raiseTower = false;
        }*/

        //U Bar
        int num = 0;
        if(gamepad.b) {
            gamepad.b = false;
            drive(	5281.1, 0.2, telemetry);
        }
        if(gamepad.a) {
            gamepad.a = false;
            drive(	5281.1, 0.2, telemetry);
        }
        if(gamepad.x) {
            gamepad.x = false;
            drive(5281.1, 0.2, telemetry);
        }
        if(gamepad.y) {
            gamepad.y = false;
            stop();
        }

        //Intake
        if(gamepad.right_bumper) {
            intakePos = true;
        }
        if(gamepad.left_bumper) {
            intakePos = false;
        }
        handleIntake();
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT