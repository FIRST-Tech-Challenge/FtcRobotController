package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

public class Robot2024 {
    Servo gripper;
    DcMotor Shoulder_Motor;

    public Robot2024(Servo gripper, DcMotor Shoulder_Motor) {
        this.gripper = gripper;
        this.Shoulder_Motor = Shoulder_Motor;
    }

    public void closeGripper(){
        gripper.setPosition(0);
    }

    public void openGripper(){
        gripper.setPosition(0.4);
    }

    public void setShoulderPower(double powerLevel){
        Shoulder_Motor.setPower(powerLevel);
    }

}
