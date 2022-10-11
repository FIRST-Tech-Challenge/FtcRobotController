package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp;

public class TowerController{

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor screw;
    public DigitalChannel highSensor;
    public DigitalChannel lowSensor;
    public boolean raiseTower;

    public TowerController (HardwareMap hardwareMap){

        //Setup motor
        screw = hardwareMap.get(DcMotor.class, "screw");
        screw.setDirection(DcMotor.Direction.FORWARD);

        //Setup sensors
        DigitalChannel highSensor = hardwareMap.get(DigitalChannel.class, "highSensor");
        DigitalChannel lowSensor = hardwareMap.get(DigitalChannel.class, "lowSensor");
        highSensor.setMode(DigitalChannel.Mode.INPUT);
        lowSensor.setMode(DigitalChannel.Mode.INPUT);
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
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //hi. you found me. -SECRET COMMENT