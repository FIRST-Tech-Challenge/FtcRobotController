package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    static final double FLIP_MIN_POS     =  0.8;     // Minimum rotational position of the wrist servo

    double rightClawPos = RIGHT_MIN_POS;
    double leftClawPos = LEFT_MAX_POS;
    double flipPos = FLIP_MAX_POS;

    public Servo rightClaw;
    public Servo leftClaw;
    public Servo intakeFlip;

    public Telemetry telemetry;

    /**
     * Gets the hardware map information of the servos.
     * @Param: rightC - the servo that controls the right of the claw
     * @Param: leftC - the servo that controls the left of the claw
     * @Param: intFlip - the servo that controls the rotation of the wrist
     */
   public IntakeMovement(Servo rightC, Servo leftC, Servo intFlip, Telemetry telemetry1) {
       rightClaw = rightC;
       leftClaw = leftC;
       intakeFlip = intFlip;
       telemetry = telemetry1;
   }

    /**
     *  Sets both the left and right servos to an open position.
     */
    public void ClawOpen(){
        while (rightClawPos<RIGHT_MAX_POS && leftClawPos>LEFT_MIN_POS) {
            rightClawPos += .01;
            leftClawPos -= .01;
            rightClaw.setPosition(rightClawPos);
            leftClaw.setPosition(leftClawPos);
            telemetry.addData("right claw", rightClawPos);
            telemetry.addData("left claw", leftClawPos);
            System.out.println(rightClawPos);
            System.out.println(rightClaw.getPosition());
            System.out.println(leftClawPos);
            System.out.println(leftClaw.getPosition());
            telemetry.update();
        }
    }

    /**
     * Sets both the left and right servos to a closed position.
     */
    public void ClawClosed(){
        while (rightClawPos>RIGHT_MIN_POS && leftClawPos<LEFT_MAX_POS) {
            rightClawPos -= .01;
            leftClawPos += .01;
            rightClaw.setPosition(rightClawPos);
            leftClaw.setPosition(leftClawPos);
            telemetry.addData("right claw", rightClawPos);
            telemetry.addData("left claw", leftClawPos);
            telemetry.update();
            System.out.println(rightClawPos);
            System.out.println(rightClaw.getPosition());
            System.out.println(leftClawPos);
            System.out.println(leftClaw.getPosition());
        }
    }

    /**
     * Sets the wrist position downward past the floor to allow
     * the claw servos to touch it.
     */
    public void FlipDown() {
        while (flipPos < FLIP_MAX_POS){
            flipPos += .01;
            intakeFlip.setPosition(flipPos);
            telemetry.addData("flip pos", flipPos);
            telemetry.update();
            System.out.println(flipPos);
            System.out.println(intakeFlip.getPosition());
        }
    }

    /**
     * Sets the wrist position upward about 180 degrees from the floor.
     */
    public void FlipUp(){
        while (flipPos>FLIP_MIN_POS) {
            flipPos -= .01;
            intakeFlip.setPosition(flipPos);
            telemetry.addData("flip pos", flipPos);
            telemetry.update();
            System.out.println(flipPos);
            System.out.println(intakeFlip.getPosition());
        }
    }

}
