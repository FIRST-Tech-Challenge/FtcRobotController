package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    static final int WRIST_UP_TICKS     =  0;     // Gge MUST be started at all times with the claw fully raised and over the conveyor.
    static final int WRIST_SAFETY_TICKS = 30;     // Gge needs to have a safety  position for the wrist to allow the linear slides to raise.
    public static final int WRIST_DOWN_TICKS   =  144;     // The number of ticks that achieve ground position for the wrist.
    double WRIST_POWER = 1.0; // Set the power of the wrist movement

    static final double SERVO_CLAW_DELAY = 1.0;

    static final double CONVEYOR_ADVANCE_DELAY = 0.9;

    // Tracks the amount of time a servo moving into position will take
    private ElapsedTime servoRuntime = new ElapsedTime();
    private ElapsedTime conveyorRuntime = new ElapsedTime();

    double rightClawPos = RIGHT_MIN_POS; // Tracks the right claw position
    double leftClawPos = LEFT_MAX_POS; // Tracks the left claw position
    int flipUpTicks = WRIST_UP_TICKS;
    int flipDownTicks = WRIST_DOWN_TICKS;

    public DcMotorEx wrist;
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo conveyor;

    public Telemetry telemetry;

    public boolean intakeIsSafe = false;

    /**
     * Gets the hardware map information of the servos.
     * @Param: rightC - the servo that controls the right of the claw
     * @Param: leftC - the servo that controls the left of the claw
     * @Param: intFlip - the servo that controls the rotation of the wrist
     * @Param: ConV - the servo that controls the conveyor motion
     * @Param: telemetry1 - the controlling class to display information on the drive station screen
     */
   public IntakeMovement(Servo rightC, Servo leftC, DcMotorEx intakeFlip, Servo conV, Telemetry telemetry1) {
       rightClaw = rightC;
       leftClaw = leftC;
       wrist = intakeFlip;
       conveyor = conV;
       telemetry = telemetry1;
       initWrist();
   }

    /**
     * Resets the DcMotor powered wrist encoder positions to 0
     */
    private void initWrist(){
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
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
     * Create a generic method to move the DcMotor powered wrist to a set position (in ticks)
     */
    public void moveWrist(int ticks, double power){
        wrist.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        wrist.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wrist.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (wrist.getTargetPosition() - wrist.getCurrentPosition()) > 5){
            telemetry.addData("Wrist Target Position...", wrist.getTargetPosition());
            telemetry.addData("Wrist Movement Power...", power);
            telemetry.addData("Wrist Position Now...", wrist.getCurrentPosition());
            telemetry.update();
        }
    }


    /**
     * Sets the wrist position downward past the floor to allow
     * the claw servos to touch it.
     */
    public void FlipDown() {
        wrist.setVelocityPIDFCoefficients(45, 0.1, 0.001, 4);
        moveWrist (WRIST_DOWN_TICKS, WRIST_POWER);
    }

    /**
     * Sets the wrist position downward past the floor to allow
     * the claw servos to touch it.
     */
    public void FlipSafety() {
        // For Wrist, PIDF values set to reduce jitter
        wrist.setVelocityPIDFCoefficients(20, 0, 0, 2);
        moveWrist (WRIST_SAFETY_TICKS, WRIST_POWER);
    }

    /**
     * Sets the wrist position upward about 180 degrees from the floor.
     */
    public void FlipUp(){
        // For Wrist, PIDF values set to reduce jitter
        wrist.setVelocityPIDFCoefficients(25, 0.3, 0.02, 16);
        moveWrist (WRIST_UP_TICKS, WRIST_POWER);

    }

    /**
     * A higher level "ease of use" fusion of several functions designed to let the
     * driver open the claw, lower the claw, close the claw, flip the claw, drop a
     * gamepiece and return to the FLIP_MID_POS in a single step.
     */
    public void GrabAndStowPixel() {
        intakeIsSafe = false;

        double servoMoveEndTime = SERVO_CLAW_DELAY + servoRuntime.time();
        while (servoMoveEndTime >= servoRuntime.time()){
            telemetry.addData("Running GrabAndSlowSequence...", WRIST_UP_TICKS);
            telemetry.update();
            ClawClosed();
            FlipUp();
            ClawOpen();
        }

        intakeIsSafe = true;
    }

    /**
     *  Advance the conveyor for CONVEYOR_ADVANCE_DELAY to move the pixel out of the way to ready
     *  for the next pixel.
     */
    public void AdvanceConveyor(){
        double conveyorMoveEndTime = CONVEYOR_ADVANCE_DELAY + conveyorRuntime.time();
        while (conveyorMoveEndTime >= conveyorRuntime.time()) {
            conveyor.setPosition(0);
        }
        conveyor.setPosition(0.5);
    }

    /**
     *  Reverse the conveyor for 2/3 of CONVEYOR_ADVANCE_DELAY in the event that a pixel GrabandStow
     *  resulted in the a missed pixel.
     */
    public void ReverseConveyor(){
        double conveyorMoveEndTime = (0.66 * CONVEYOR_ADVANCE_DELAY) + conveyorRuntime.time();
        while (conveyorMoveEndTime >= conveyorRuntime.time()) {
            conveyor.setPosition(1);
        }
        conveyor.setPosition(0.5);
    }

}
