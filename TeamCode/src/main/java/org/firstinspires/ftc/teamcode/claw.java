package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class claw {
    //input hardware//
    DcMotor input_slides;
    Servo input_arm_pitch_left;
    Servo input_arm_pitch_right;
    Servo input_elbow_pitch;
    Servo input_wrist_roll;
    Servo input_claw;
    //output hardware//
    DcMotor output_slides_left;
    DcMotor output_slides_right;
    Servo output_elbow_pitch_right;
    Servo output_elbow_pitch_left;
    Servo output_wrist_roll;
    Servo output_claw;
    public void clawHW(HardwareMap hwm, Telemetry tm, Gamepad gp){

    }
}
