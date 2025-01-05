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
        input_slides = hwm.dcMotor.get(HMapConfig.INPUT_SLIDES);
        input_arm_pitch_left = hwm.servo.get(HMapConfig.INPUT_ARM_PITCH_LEFT);
        input_arm_pitch_right = hwm.servo.get(HMapConfig.INPUT_ARM_PITCH_RIGHT);
        input_elbow_pitch = hwm.servo.get(HMapConfig.INPUT_ELBOW_PITCH);
        input_wrist_roll = hwm.servo.get(HMapConfig.INPUT_WRIST_ROLL);
        input_claw = hwm.servo.get(HMapConfig.INPUT_CLAW);
        output_slides_left = hwm.dcMotor.get(HMapConfig.OUTPUT_SLIDES_LEFT);
        output_slides_right = hwm.dcMotor.get(HMapConfig.OUTPUT_SLIDES_RIGHT);
        output_elbow_pitch_right = hwm.servo.get(HMapConfig.OUTPUT_ELBOW_PITCH_RIGHT);
        output_elbow_pitch_left = hwm.servo.get(HMapConfig.OUTPUT_ELBOW_PITCH_LEFT);
        output_wrist_roll = hwm.servo.get(HMapConfig.OUTPUT_WRIST_ROLL);
        output_claw = hwm.servo.get(HMapConfig.OUTPUT_CLAW);
    }
}
