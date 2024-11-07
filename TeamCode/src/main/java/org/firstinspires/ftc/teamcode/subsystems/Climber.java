package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber extends SubsystemBase {

    private ServoEx servo;

    public Climber(HardwareMap hardwareMap){
        //TODO: fix this name from config
        servo = new SimpleServo(hardwareMap, "clamp", 0, 360);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void lift(){
        //this is a guess
        setAngle(360);
    }
    public void lower(){
        //this is a guess
        setAngle(0);
    }

    public void center(){
        //this is a guess
        setAngle(180);
    }

    @Override
    public void periodic(){
    }

}