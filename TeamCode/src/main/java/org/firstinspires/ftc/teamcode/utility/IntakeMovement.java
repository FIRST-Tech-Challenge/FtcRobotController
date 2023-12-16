package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    static final double FLIP_MAX_POS     =  0.99;     // Maximum rotational position of the wrist servo
    static final double FLIP_MIN_POS     =  0.35;     // Was 0.35 Minimum rotational position of the wrist servo
    static final double SERVO_CLAW_DELAY = 0.6;

    static final double SERVO_FLIP_DELAY = 0.5;

    // Tracks the amount of time a servo moving into position will take
    static final double servoDelayTime = 0.5; // safety delay for servo move

    private ElapsedTime servoRuntime = new ElapsedTime();


    double rightClawPos = RIGHT_MIN_POS; // Tracks the right claw position
    double leftClawPos = LEFT_MAX_POS; // Tracks the left claw position
    double flipPos = FLIP_MAX_POS;

    public Servo rightClaw;
    public Servo leftClaw;
    public Servo intakeFlip;

    public Telemetry telemetry;

    public boolean intakeIsSafe = false;

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

   public boolean setSafety(){
       return intakeIsSafe;
   }
    /**
     *  Sets both the left and right servos to an open position.
     */
    public void ClawOpen(){
        intakeIsSafe = false;

        double servoMoveEndTime = SERVO_CLAW_DELAY + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()) {
            rightClaw.setPosition(RIGHT_MAX_POS);
            leftClaw.setPosition(LEFT_MIN_POS);
        }

        intakeIsSafe = true;

    }

    /**
     * Sets both the left and right servos to a closed position.
     */
    public void ClawClosed(){
        intakeIsSafe = false;

        double servoMoveEndTime = SERVO_CLAW_DELAY + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()) {
            rightClaw.setPosition((RIGHT_MIN_POS));
            leftClaw.setPosition(LEFT_MAX_POS);
        }
        intakeIsSafe = true;

    }

    /**
     * Sets the wrist position downward past the floor to allow
     * the claw servos to touch it.
     */
    public void FlipDown() {
        intakeIsSafe = false;

        double servoMoveEndTime = SERVO_FLIP_DELAY + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()){
            intakeFlip.setPosition(FLIP_MAX_POS);
            telemetry.addData("flip pos", FLIP_MAX_POS);
            telemetry.update();
        }

        intakeIsSafe = true;
    }

    /**
     * Sets the wrist position upward about 180 degrees from the floor.
     */
    public void FlipUp(){
        intakeIsSafe = false;

        double servoMoveEndTime = SERVO_FLIP_DELAY + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()){
            intakeFlip.setPosition(FLIP_MIN_POS);
            telemetry.addData("flip pos", FLIP_MIN_POS);
            telemetry.update();
        }
        intakeIsSafe = true;

    }

    /**
     * A higher level "ease of use" fusion of several functions designed to let the
     * driver open the claw, lower the claw, close the claw, flip the claw, drop a
     * gamepiece and return to the FLIP_MID_POS in a single step.
     */
    public void GrabAndStowPixel() {
        intakeIsSafe = false;

        double servoMoveEndTime = servoDelayTime + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()){
            telemetry.addData("Running GrabAndSlowSequence...", FLIP_MIN_POS);
            telemetry.update();
            //ClawOpen(); // This automation was removed because the drive team found it useful to
            //FlipDown(); // lower the wrist and plow into the pixel before trying to close the
            //sleep(500); // claw.  Will still Grab and Stow but wont Lower Grab and Stow.
            ClawClosed();
            //sleep(500);
            FlipUp();
            //sleep(500);
            ClawOpen();
        }

        intakeIsSafe = true;
    }

}
