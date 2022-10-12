package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp;

public class TowerController{

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor screw;
    public DcMotor uBar;
    public DigitalChannel highSensor;
    public DigitalChannel lowSensor;
    public DigitalChannel uBarSensor;
    public boolean raiseTower;

    public int uBarLevel;
    static final double     COUNTS_PER_MOTOR    = 384.5;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION);

    public TowerController (HardwareMap hardwareMap){

        //Setup motors
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");
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
        drive(COUNTS_PER_INCH * 8, 1);

    }

    private void drive(double uBarTarget, double speed) {
        uBarLevel += uBarTarget;
        uBar.setTargetPosition(uBarLevel);
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT