package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TowerController {

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor screw;
    private DcMotor uBar;
    private Servo intake;
    private DigitalChannel highSensor;
    private DigitalChannel lowSensor;
    public boolean raiseTower;
    public boolean intakePos;
    public double uBarNum = 1;

    public int uBarLevel;
    static final double     COUNTS_PER_MOTOR    = 384.5;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION);

    public TowerController (HardwareMap hardwareMap){

        //Setup motors
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
        intake = hardwareMap.get(Servo.class, "intake");
        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.FORWARD);

        //Setup sensors
        DigitalChannel highSensor = hardwareMap.get(DigitalChannel.class, "highSensor");
        DigitalChannel lowSensor = hardwareMap.get(DigitalChannel.class, "lowSensor");
        highSensor.setMode(DigitalChannel.Mode.INPUT);
        lowSensor.setMode(DigitalChannel.Mode.INPUT);

        //setup encoder
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void handleScrew() {
        if(raiseTower){
            if(highSensor.getState()){
                screw.setPower(0);
            }
            else{
                screw.setPower(1);
            }
        }
        else{
            if(lowSensor.getState()){
                screw.setPower(0);
            }
            else{
                screw.setPower(-1);
            }
        }
    }

    public void handleUBar() {
        //set U bar to position depending on the uBarNum variable
        if (uBarNum == 1) {
            drive((int) COUNTS_PER_INCH * 20, 1);
        }
        if (uBarNum == 2) {
            drive((int) COUNTS_PER_INCH * 40, 1);
        }
        if (uBarNum == 3) {
            drive((int) COUNTS_PER_INCH * 60, 1);
        }
        if (uBarNum == 4) {
            drive((int) COUNTS_PER_INCH * 80, 1);
        }
    }

    private void drive(int uBarTarget, double speed) {
        uBarLevel = uBarTarget;
        uBar.setTargetPosition(uBarLevel);
    }

    public void handleIntake () {
        if(intakePos){
            intake.setPosition(0.5);
        }
        else{
            intake.setPosition(0);
        }
    }
    public void handleGamepad(Gamepad gamepad) {

        //Screw
        /* if(gamepad.dpad_up) {
            raiseTower = true;
        }
        if(gamepad.dpad_down) {
            raiseTower = false;
        }*/

        //U Bar
        if(gamepad.b) {
            uBarLevel = 1;
        }
        if(gamepad.a) {
            uBarLevel = 2;
        }
        if(gamepad.x) {
            uBarLevel = 3;
        }if(gamepad.y) {
            uBarLevel = 4;
        }

        //Intake
        if(gamepad.right_bumper) {
            intakePos = true;
        }
        if(gamepad.left_bumper) {
            intakePos = false;
        }
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT