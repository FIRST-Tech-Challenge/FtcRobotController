package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Grabber extends SubsystemBase {

    private CRServo servo;

    public Grabber(HardwareMap hardwareMap){
        //TODO: fix this name from config
        servo = hardwareMap.crservo.get("grabber");
    }

    public void pickup() {
        servo.setPower(-0.5);
    }

    public void drop() {
        servo.setPower(-0.5);
    }

    @Override
    public void periodic(){
    }

}