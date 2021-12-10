package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.teamcode.Constants.hopperBottom;
import static org.firstinspires.ftc.teamcode.Constants.hopperTop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hopper implements Mechanism {
    public Servo hopper;
    int state = 1;
    @Override
    public void init(HardwareMap hardwareMap) {
        hopper = hardwareMap.get(Servo.class, "hopper");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.a) {
            //Make sure it doesn't set the position constantly
            if(state != 1) {
                hopper.setPosition(hopperTop);
                state = 1;
            }
        } else {
            //Make sure it doesn't set the position constantly
            if(state != 0) {
                hopper.setPosition(hopperBottom);
                state = 0;
            }
        }
    }
}
