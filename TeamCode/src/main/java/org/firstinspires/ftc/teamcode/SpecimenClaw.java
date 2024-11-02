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
        if( gamepad.left_bumper) { releaseSpecimen(); }
        else if( gamepad.right_bumper){ grabSpecimen(); }

        //if(gamepad.left_bumper){ clampDown = !clampDown; }
    }

    public void setLeftPosition(double d){
        leftClawServo.setPosition(d);
        //rightClawServo.setPosition(1-d);
    }

    public void setRightPosition(double d) {
        rightClawServo.setPosition(d);
    }
    public void grabSpecimen(){
        leftClawServo.setPosition(0.562);
        rightClawServo.setPosition(0.162);
    }

    public void releaseSpecimen(){
        leftClawServo.setPosition(0.562-0.4);
        rightClawServo.setPosition(0.162+0.4);
    }
}
