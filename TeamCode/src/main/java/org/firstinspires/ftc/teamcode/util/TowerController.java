package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp;

public class TowerController {

    //setup variables, motors, and servos
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor screw;
    public DcMotor uBar;
    public Servo intake;
    public DigitalChannel highSensor;
    public DigitalChannel lowSensor;
    public DigitalChannel uBarSensor;
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
        DigitalChannel uBarSensor = hardwareMap.get(DigitalChannel.class, "uBarSensor");
        highSensor.setMode(DigitalChannel.Mode.INPUT);
        lowSensor.setMode(DigitalChannel.Mode.INPUT);
        uBarSensor.setMode(DigitalChannel.Mode.INPUT);


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
            drive(COUNTS_PER_INCH * 20, 1);
        }
        if (uBarNum == 2) {
            drive(COUNTS_PER_INCH * 40, 1);
        }
        if (uBarNum == 3) {
            drive(COUNTS_PER_INCH * 60, 1);
        }
        if (uBarNum == 4) {
            drive(COUNTS_PER_INCH * 80, 1);
        }
    }

    private void drive(double uBarTarget, double speed) {
        uBarLevel += uBarTarget;
        uBar.setTargetPosition(uBarLevel);
    }

    public void handleIntake (HardwareMap hardwareMap) {
        if(intakePos){
            intake.setPosition(1);
        }
        else{
            intake.setPosition(0);
        }
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT