package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeMovement {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    // Right claw
    static final double RIGHT_MAX_POS     =  0.4;     // Maximum rotational position of the right servo
    static final double RIGHT_MIN_POS     =  0.0;     // Minimum rotational position of the right servo
    // Left claw
    static final double LEFT_MAX_POS     =  1.0;     // Maximum rotational position of the left servo
    static final double LEFT_MIN_POS     =  0.6;     // Minimum rotational position of the left servo
    // Intake flip
    static final double FLIP_MAX_POS     =  1.0;     // Maximum rotational position of the intake flip servo
    static final double FLIP_MIN_POS     =  0.35;     // Minimum rotational position intake flip servo

    public Servo rightClaw;
    public Servo leftClaw;
    public Servo intakeFlip;


   public IntakeMovement(Servo rightC, Servo leftC, Servo intFlip) {
       rightClaw = rightC;
       leftClaw = leftC;
       intakeFlip = intFlip;
   }
    public void ClawOpen(){
        rightClaw.setPosition(RIGHT_MAX_POS);
        leftClaw.setPosition(LEFT_MIN_POS);
    }

    public void ClawClosed(){
        rightClaw.setPosition(RIGHT_MIN_POS);
        leftClaw.setPosition(LEFT_MAX_POS);
    }

    // Flips the wrist upwards
    public void FlipDown(){
       intakeFlip.setPosition(FLIP_MAX_POS);
    }

    // Flips the wrist downwards
    public void FlipUp(){
       intakeFlip.setPosition(FLIP_MIN_POS);
    }

}
