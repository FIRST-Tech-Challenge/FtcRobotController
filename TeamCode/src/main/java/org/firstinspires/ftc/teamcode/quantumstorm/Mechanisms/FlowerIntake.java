package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Flower intake arm on a goBILDA Dual Mode Servo (servo mode), commanded in arm degrees.
 * Shared by QuantumStormTeleOp and autonomous. The servo does not move until the
 * first command, so call lift() during INIT to put the arm in a known place.
 */
public class FlowerIntake {

    private final ServoImplEx servo;
    private double angle = Constants.FLOWER_UP_DEG;

    // Combos held last loop, so each press acts once
    private boolean previousLift = false;
    private boolean previousDrop = false;
    private boolean previousNudgeUp = false;
    private boolean previousNudgeDown = false;

    public FlowerIntake(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, Constants.FLOWER_SERVO);

        // goBILDA Dual Mode Servo accepts a wider pulse than the hub's 600..2400 us default
        servo.setPwmRange(new PwmControl.PwmRange(
                Constants.FLOWER_SERVO_PWM_MIN_US,
                Constants.FLOWER_SERVO_PWM_MAX_US));

        servo.setDirection(Constants.FLOWER_SERVO_REVERSED
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD);
    }

    public void drop() {
        setAngle(Constants.FLOWER_DOWN_DEG);
    }

    public void lift() {
        setAngle(Constants.FLOWER_UP_DEG);
    }

    /** Moves the arm, kept inside FLOWER_ANGLE_MIN_DEG..FLOWER_ANGLE_MAX_DEG. */
    public void setAngle(double degrees) {
        angle = Math.max(Constants.FLOWER_ANGLE_MIN_DEG,
                Math.min(Constants.FLOWER_ANGLE_MAX_DEG, degrees));

        servo.setPosition(Constants.FLOWER_SERVO_ZERO_POSITION
                + angle / Constants.FLOWER_SERVO_RANGE_DEG);
    }

    /**
     * Driver controls (gamepad 2 in TeleOp), called once per loop:
     *   BACK + D-pad UP    = lift to FLOWER_UP_DEG
     *   BACK + D-pad DOWN  = drop to FLOWER_DOWN_DEG
     *   LB + D-pad UP/DOWN = nudge by FLOWER_NUDGE_DEG (tuning)
     * Acts when the whole combo becomes true, so the buttons can be pressed
     * in any order or together. The D-pad alone never moves the arm.
     */
    public void handleGamepad(Gamepad gamepad) {

        boolean up = gamepad.dpad_up && !gamepad.dpad_down;
        boolean down = gamepad.dpad_down && !gamepad.dpad_up;

        boolean lift = gamepad.back && up;
        boolean drop = gamepad.back && down;
        boolean nudgeUp = !gamepad.back && gamepad.left_bumper && up;
        boolean nudgeDown = !gamepad.back && gamepad.left_bumper && down;

        if (lift && !previousLift) lift();
        if (drop && !previousDrop) drop();
        if (nudgeUp && !previousNudgeUp) setAngle(angle + Constants.FLOWER_NUDGE_DEG);
        if (nudgeDown && !previousNudgeDown) setAngle(angle - Constants.FLOWER_NUDGE_DEG);

        previousLift = lift;
        previousDrop = drop;
        previousNudgeUp = nudgeUp;
        previousNudgeDown = nudgeDown;
    }

    public double getAngle() {
        return angle;
    }

    /** True when the arm is closer to DOWN than UP. */
    public boolean isDown() {
        return angle < (Constants.FLOWER_UP_DEG + Constants.FLOWER_DOWN_DEG) / 2;
    }

    /** Raw 0..1 servo position, for tuning FLOWER_SERVO_ZERO_POSITION. */
    public double getPosition() {
        return servo.getPosition();
    }
}
