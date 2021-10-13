package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    private DcMotor linearSlide;
    private Servo grabberServo;

    private static final float openPosition  = 0.5f;
    private static final float closePosition = .95f;


    public Grabber(HardwareMap hardwareMap, String servoName){
        grabberServo = hardwareMap.servo.get(servoName);
    }

    public void open(){
        grabberServo.setPosition(openPosition);
    }

    public void close(){
        grabberServo.setPosition(closePosition);
    }

    public boolean isOpen(){
         if (grabberServo.getPosition() == openPosition){
            return true;
        }else {
            return false;
        }
    }

    public boolean isClosed(){
        if (grabberServo.getPosition() == closePosition){
            return true;
        }else {
            return false;
        }
    }
}
