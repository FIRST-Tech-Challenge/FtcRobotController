package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Flower Picker Subsystem (issue #75)
 * Owns the two FLOWER PICKER servos that move the "flower bar". Only this class touches those
 * servos. OpModes call its methods.
 *
 * The flower bar has TWO places it can be:
 *   UP   -> the bar is lifted out of the way.
 *   DOWN -> the bar is lowered, ready to pick a flower.
 *
 * goToUp() and goToDown() pick one on purpose. Both are safe to call every loop.
 *
 * TWO SERVOS, MIRRORED
 *   One servo is on each side of the bar, and they face opposite ways. So to move the bar, one
 *   servo has to turn CLOCKWISE while the other turns COUNTER-CLOCKWISE. Their position numbers
 *   are therefore different: where the left servo goes to 0.30, the right one goes to about 0.70
 *   (1.0 minus 0.30). We keep FOUR separate numbers (left UP, left DOWN, right UP, right DOWN)
 *   instead of using setDirection(REVERSE). That way every position is written out where a
 *   student can see it and change it, and the two sides can be tuned on their own.
 *
 * STATE (what we told the bar to do) is UP or DOWN.
 * A servo cannot tell us where it really is, so there is no sensor STATUS here.
 * The closest thing is isAtPosition(): it is true once SERVO_TRAVEL_SECONDS have passed since the
 * last command, which is our guess that the bar has finished moving.
 *
 * Nothing needs update(): the bar only needs a timer, and the timer runs by itself.
 *
 * Robot configuration (on the Driver Station):
 *   Servo name: left_flower_servo    (plain "Servo", POSITIONAL mode)
 *   Servo name: right_flower_servo   (plain "Servo", POSITIONAL mode)
 * Both servos must be in POSITIONAL mode (not continuous), or these position commands would spin
 * them instead.
 */
public class FlowerPickerSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String LEFT_SERVO_NAME = "left_flower_servo";
    private static final String RIGHT_SERVO_NAME = "right_flower_servo";

    // ------------------------------------------------------------------
    // Bar positions
    // ------------------------------------------------------------------
    // A servo position is a number from 0.0 to 1.0. That is the servo's whole travel:
    // 0.0 is one end and 1.0 is the other end.
    //
    // The two servos are mirrored, so the right numbers start as "1.0 minus the left number".
    // If the bar moves the WRONG way (DOWN when you asked UP), swap the UP and DOWN numbers.
    // If only ONE side moves the wrong way, swap the numbers for that side only.
    //
    // TODO (issue #75): STARTING GUESSES, never tried on the robot. Find the real numbers with
    // FlowerPickerAlpha: put the robot on blocks, creep each servo with the nudge buttons until the
    // bar is exactly where you want it (the OpMode shows the number), then type the numbers in here.
    // 11940 and 22091 may end up with different numbers. Per-robot numbers wait for the build
    // flavors (issue #66), so for now there is one set.
    private static final double LEFT_UP_POSITION = 0.30;
    private static final double LEFT_DOWN_POSITION = 0.70;
    private static final double RIGHT_UP_POSITION = 0.70;
    private static final double RIGHT_DOWN_POSITION = 0.30;

    // The servos do not tell us when they arrive. This is how long we assume a move takes.
    // Too short and isAtPosition() lies. Too long and we wait for nothing.
    // TODO (issue #75): Starting guess. Time the bar with a phone stopwatch, then change this number.
    private static final double SERVO_TRAVEL_SECONDS = 0.50;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = where we told the bar to go.
    public enum PickerState {
        UP,     // Bar is at (or moving to) the UP position.
        DOWN    // Bar is at (or moving to) the DOWN position.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private Servo leftFlowerServo;
    private Servo rightFlowerServo;

    private PickerState state = PickerState.UP;

    // These start at the constants above. They are variables (not constants) only so
    // nudgeLeftPosition() and nudgeRightPosition() can change them while we are finding the
    // right numbers.
    private double leftUpPosition = LEFT_UP_POSITION;
    private double leftDownPosition = LEFT_DOWN_POSITION;
    private double rightUpPosition = RIGHT_UP_POSITION;
    private double rightDownPosition = RIGHT_DOWN_POSITION;

    // Times how long ago we last gave the servos a new position.
    private ElapsedTime travelTimer = new ElapsedTime();

    // False until we send the servos their first position (in the constructor).
    private boolean servosHaveBeenCommanded = false;

    /*
     * Setup. Gets both servos from the robot configuration and moves the bar to the UP
     * position, so we always start from a known place.
     * (Rule R103.B allows an OpMode to pre-position servos at INIT.)
     */
    public FlowerPickerSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        leftFlowerServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightFlowerServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        goToUp();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the bar what to do
    // ==================================================================

    /*
     * Move the bar to the UP position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToUp() {
        setState(PickerState.UP);
    }

    /*
     * Move the bar to the DOWN position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToDown() {
        setState(PickerState.DOWN);
    }

    /*
     * For finding the right positions. Moves the LEFT servo's position for the CURRENT state
     * (UP or DOWN) by a small amount, and sends the servos there.
     * A positive amount moves toward 1.0, a negative amount toward 0.0.
     * The result is kept between 0.0 and 1.0.
     *
     * This only changes the number while the OpMode runs. When you find a good number, you still
     * have to type it into LEFT_UP_POSITION or LEFT_DOWN_POSITION above.
     */
    public void nudgeLeftPosition(double amount) {
        if (state == PickerState.UP) {
            leftUpPosition = Range.clip(leftUpPosition + amount, 0.0, 1.0);
        } else {
            leftDownPosition = Range.clip(leftDownPosition + amount, 0.0, 1.0);
        }
        sendServosToCurrentState();
    }

    /*
     * The same as nudgeLeftPosition(), but for the RIGHT servo.
     * When you find a good number, type it into RIGHT_UP_POSITION or RIGHT_DOWN_POSITION above.
     */
    public void nudgeRightPosition(double amount) {
        if (state == PickerState.UP) {
            rightUpPosition = Range.clip(rightUpPosition + amount, 0.0, 1.0);
        } else {
            rightDownPosition = Range.clip(rightDownPosition + amount, 0.0, 1.0);
        }
        sendServosToCurrentState();
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the bar what is happening
    // ==================================================================

    public PickerState getState() {
        return state;
    }

    public boolean isUp() {
        return state == PickerState.UP;
    }

    public boolean isDown() {
        return state == PickerState.DOWN;
    }

    /*
     * True once enough time has passed for the bar to finish its move.
     * This is a TIME GUESS, not a sensor reading.
     */
    public boolean isAtPosition() {
        return travelTimer.seconds() >= SERVO_TRAVEL_SECONDS;
    }

    // The position (0.0 to 1.0) we are currently sending to the LEFT servo.
    public double getLeftTargetPosition() {
        if (state == PickerState.UP) {
            return leftUpPosition;
        }
        return leftDownPosition;
    }

    // The position (0.0 to 1.0) we are currently sending to the RIGHT servo.
    public double getRightTargetPosition() {
        if (state == PickerState.UP) {
            return rightUpPosition;
        }
        return rightDownPosition;
    }

    /*
     * Adds the flower picker's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- FLOWER PICKER ---");
        telemetry.addData("Picker State", state);
        telemetry.addData("Left Servo Position", "%.3f", getLeftTargetPosition());
        telemetry.addData("Right Servo Position", "%.3f", getRightTargetPosition());
        telemetry.addData("Picker At Position?", isAtPosition());
        telemetry.addData("Left UP / DOWN", "%.3f / %.3f", leftUpPosition, leftDownPosition);
        telemetry.addData("Right UP / DOWN", "%.3f / %.3f", rightUpPosition, rightDownPosition);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    // Change the state and send the servos there. If we are ALREADY in that state, do
    // nothing, so a held button will not keep restarting the travel timer.
    private void setState(PickerState newState) {
        // The very first call (from the constructor) must always go through, even though
        // the state already says UP. servosHaveBeenCommanded is false until then.
        if (servosHaveBeenCommanded && newState == state) {
            return;
        }
        state = newState;
        sendServosToCurrentState();
    }

    private void sendServosToCurrentState() {
        leftFlowerServo.setPosition(getLeftTargetPosition());
        rightFlowerServo.setPosition(getRightTargetPosition());
        servosHaveBeenCommanded = true;
        travelTimer.reset();
    }
}
