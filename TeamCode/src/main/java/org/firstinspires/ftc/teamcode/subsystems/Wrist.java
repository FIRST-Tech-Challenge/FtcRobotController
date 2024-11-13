package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends SubsystemBase {

    private ServoEx servo;
    private Telemetry tm;


    public Wrist(HardwareMap hardwareMap, Telemetry telemetry){
        tm = telemetry;
        //TODO: fix this name from config
        servo = new SimpleServo(hardwareMap, "wrist", 0, 300);
        servo.setInverted(false);
    }

    public void addFifteen() {
        servo.rotateBy(15);
    }
    public void subFifteen() {
        servo.rotateBy(-15);
    }
    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void open(){
        setAngle(200);
    }
    public void close(){
        setAngle(65);
    }

    @Override
    public void periodic(){
    }

}