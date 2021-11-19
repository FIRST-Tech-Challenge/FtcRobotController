package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hopper implements Mechanism {
    Servo hopper;
    int state = 1;
    @Override
    public void init(HardwareMap hardwareMap) {
        hopper = hardwareMap.get(Servo.class, "hopper");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.5) {
            //Make sure it doesn't set the position constantly
            if(state != 1) {
                hopper.setPosition(0.3);
                state = 1;
            }
        } else {
            //Make sure it doesn't set the position constantly
            if(state != 0) {
                hopper.setPosition(0);
                state = 0;
            }
        }
    }
}
