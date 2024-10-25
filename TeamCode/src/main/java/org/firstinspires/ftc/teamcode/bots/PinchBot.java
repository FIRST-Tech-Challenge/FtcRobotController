package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.hardware.Servo;

public class PinchBot extends PivotBot{

    public Servo pinch;
    public Servo rotate;

    public PinchBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        pinch = hardwareMap.get(Servo.class, "pinch");
        rotate = hardwareMap.get(Servo.class, "rotate");
    }

    public void PinchControl(boolean open, boolean close){
        if(open){
            pinch.setPosition(0.5);
        }
        if(close){
            pinch.setPosition(0);
        }
    }
    public void rotate(double angle){

        rotate.setPosition(angle);
    }

}
