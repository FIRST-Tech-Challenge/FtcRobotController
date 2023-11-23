package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class contains methods that control our claw and wrist intake system.
 */
public class IntakeMovement {

    // Right claw
    static final double RIGHT_MAX_POS     =  0.4;     // Maximum rotational position of the right servo
    static final double RIGHT_MIN_POS     =  0.0;     // Minimum rotational position of the right servo
    // Left claw
    static final double LEFT_MAX_POS     =  1.0;     // Maximum rotational position of the left servo
    static final double LEFT_MIN_POS     =  0.6;     // Minimum rotational position of the left servo
    // Intake flip
    // The lower the int the higher the wrist goes
    static final double FLIP_MAX_POS     =  1.0;     // Maximum rotational position of the wrist servo
    static final double FLIP_MIN_POS     =  0.4;     // Minimum rotational position of the wrist servo


    public Servo rightClaw;
    public Servo leftClaw;
    public Servo intakeFlip;

    /**
     * Gets the hardware map information of the servos.
     * @Param: rightC - the servo that controls the right of the claw
     * @Param: leftC - the servo that controls the left of the claw
     * @Param: intFlip - the servo that controls the rotation of the wrist
     */
   public IntakeMovement(Servo rightC, Servo leftC, Servo intFlip) {
       rightClaw = rightC;
       leftClaw = leftC;
       intakeFlip = intFlip;
   }

    /**
     *  Sets both the left and right servos to an open position.
     */
    public void ClawOpen(){
        rightClaw.setPosition(RIGHT_MAX_POS);
        leftClaw.setPosition(LEFT_MIN_POS);
    }

    /**
     * Sets both the left and right servos to a closed position.
     */
    public void ClawClosed(){
        rightClaw.setPosition(RIGHT_MIN_POS);
        leftClaw.setPosition(LEFT_MAX_POS);
    }

    /**
     * Sets the wrist position downward past the floor to allow
     * the claw servos to touch it.
     */
    public void FlipDown(){
       intakeFlip.setPosition(FLIP_MAX_POS);
    }

    /**
     * Sets the wrist position upward about 180 degrees from the floor.
     */
    public void FlipUp(){
       intakeFlip.setPosition(FLIP_MIN_POS);
    }

}
