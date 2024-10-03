package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

public class Robot2024 {
    Servo gripper;
    DcMotor Shoulder_Motor;
    double gripper_open_position = 0.0;
    double gripper_close_position = 0.3;

    public Robot2024(Servo gripper, DcMotor Shoulder_Motor) {
        this.gripper = gripper;
        this.Shoulder_Motor = Shoulder_Motor;
    }

    public void closeGripper(){
        gripper.setPosition(this.gripper_close_position);
    }

    public void openGripper(){
        gripper.setPosition(this.gripper_close_position);
    }

    public void setShoulderPower(double powerLevel){
        Shoulder_Motor.setPower(powerLevel);
    }

}
