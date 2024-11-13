package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Climber extends SubsystemBase {

    public static volatile double LIFT_ANGLE = 360;
    public static volatile double LOWER_ANGLE = 360;
    public static volatile double CENTER_ANGLE = 360;

    private final ServoEx servo;

    public Climber(HardwareMap hardwareMap){
        //TODO: fix this name from config
        servo = new SimpleServo(hardwareMap, "clamp", 0, 360);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void lift(){
        setAngle(LIFT_ANGLE);
    }
    public void lower(){
        setAngle(LOWER_ANGLE);
    }

    public void center(){
        setAngle(CENTER_ANGLE);
    }
}