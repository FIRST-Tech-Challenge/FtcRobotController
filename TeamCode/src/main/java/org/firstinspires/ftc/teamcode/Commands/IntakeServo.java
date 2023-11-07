package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServo extends Command{
    public Servo LServo;
    public Servo RServo;
    Boolean Done = false;
    public IntakeServo(HardwareMap hardwareMap){
        LServo = hardwareMap.servo.get("LServo");
        RServo = hardwareMap.servo.get("RServo");
    }

    public void start(){
        LServo.scaleRange(0, 360);
        LServo.setPosition(1);
        RServo.scaleRange(0, 360);
        RServo.setPosition(-1);
    }

    public void execute(){
        Done = true;
    }

    public void end(){

    }

    public boolean isFinished(){
        if (Done){
            return true;
        }
        return false;
    }
}
