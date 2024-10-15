package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpecimenClaw {

    // Motor names
    private Servo leftClawServo, rightClawServo;
    private String LEFT_CLAW_NAME = "leftClaw";
    private String RIGHT_CLAW_NAME = "rightClaw";
    private boolean aDown, clampDown;
    public SpecimenClaw(HardwareMap hardwareMap){
        leftClawServo = hardwareMap.servo.get(LEFT_CLAW_NAME);
        rightClawServo = hardwareMap.servo.get(RIGHT_CLAW_NAME);
        aDown=false;
        clampDown=false;
    }
    public void update(Gamepad gamepad){
        //boolean a = gamepad.a && !aDown;
        //aDown = gamepad.a;
        if( gamepad.right_bumper && clampDown) { releaseSpecimen(); }
        else if( gamepad.right_bumper && !clampDown){ grabSpecimen(); }
    }

    public void grabSpecimen(){
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(0);
    }

    public void releaseSpecimen(){
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(1);
    }
}
