package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

public class Robot2024 {
    Servo gripper;

    public Robot2024(Servo gripper) {
        this.gripper = gripper;
    }

    public void closeGripper(){
        gripper.setPosition(0);
    }

    public void openGripper(){
        gripper.setPosition(0.4);
    }

}
