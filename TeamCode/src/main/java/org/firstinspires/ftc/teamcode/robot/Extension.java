package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


public class Extension {
    private Telemetry telemetry;

    // Three servos (plus the turret) work together to place cone to desired location
    public ServoImplEx extension;
    public ServoImplEx twoBar;
    public ServoImplEx grabber;

    static final double MM_TO_INCHES = 0.0393700787;
    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;

    // Servo Positions
    public final double EXTENSION_POSITION_HOME = 0;    // Fully retracted
    public final double EXTENSION_POSITION_MAX  = 1;    // Fully extended

    public final double TWOBAR_POSITION_HOME    = 0;    // vertical position
    public final double TWOBAR_POSITION_MAX     = 1;    // fully tilted

    public final double GRABBER_POSITION_OPEN   = 0;    // open
    public final double GRABBER_POSITION_HOLD   = 0.5;  // TODO: measure the actual position when cone in place and replace this value
    public final double GRABBER_POSITION_CLOSED = 1;    // closed

    public final double EXTENSION_MAX_REACH = 10; // TODO: measure actual value in inches and replace this value

    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        extension = (ServoImplEx) hwMap.servo.get("Extension");
        twoBar = (ServoImplEx) hwMap.servo.get("Two Bar");
        grabber = (ServoImplEx) hwMap.servo.get("Grabber");

        // Scale the operating range of Servos and set initial position
        extension.setPwmRange(new PwmControl.PwmRange(1250,2522));
        extendHome();

        twoBar.setPwmRange(new PwmControl.PwmRange(1745,2522));
        tiltDown();

        grabber.setPwmRange(new PwmControl.PwmRange(100,2522));
        grabberOpen();
    }

    /************************* EXTENSION ARM UTILITIES **************************/

    // This method is intended for Teleop mode getting speed value coming from controller (-1..1)
    // Negative speed values will retract the extension arm.
    public void extend(double speed) {
        double currentPosition = extension.getPosition();

        //scale speed value so the extension moves in increments of 10% of the range at max speed
        double targetPosition = Range.clip(currentPosition + speed*0.10, 0, 1);
        extension.setPosition(targetPosition/EXTENSION_POSITION_MAX);

        //Send telemetry message for debugging purposes
        telemetry.addData("Speed of move:","%.2f", speed);
        telemetry.addData("Extension Position:","%.2f", targetPosition);
        telemetry.update();
    }

    // Move the extension's position to the specified distance in inches
    // Predefined values can be passed by using class constants (to be defined later)
    // To go as far as it can, pass extension.EXTENSION_MAX_REACH as distance.
    // To go back home, pass 0 as distance.
    public void extendTo(double distance) {
        extension.setPosition (Range.clip(distance/EXTENSION_MAX_REACH,0,1));
    }

    // Moves the extension arm to its clearing length (if it was not already in clear)
    public void getToClear() {
        if (!isInClear()) {
            extendTo(MINIMUM_CLEARANCE_DISTANCE);
        }
    }

    // Returns true if the extension arm's current position is beyond the minimum clearance distance in inches
    Boolean isInClear() {
        double currentPosition = extension.getPosition();

        return (currentPosition * EXTENSION_MAX_REACH) >= MINIMUM_CLEARANCE_DISTANCE;
    }

    // Pulls the extension arm to its starting position (it is NOT in clear)
    public void extendHome() {
        extension.setPosition(EXTENSION_POSITION_HOME);
    }

    // Extends the arm to its maximum reach
    public void extendMax() {
        extension.setPosition(EXTENSION_POSITION_MAX);
    }


    /************************* TWO-BAR UTILITIES **************************/

    // Tilt the two-bar to its max position.
    // No micro adjustments envisioned for the two-bar
    public void tiltUp() {
        twoBar.setPosition(TWOBAR_POSITION_MAX);
    }

    // Tilt the two-bar to its starting position (vertical).
    // No micro adjustments envisioned for the two-bar
    public void tiltDown() {
        twoBar.setPosition(TWOBAR_POSITION_HOME);
    }

    /************************* GRABBER UTILITIES **************************/

    // Opens the claw
    public void grabberOpen() {
        grabber.setPosition(GRABBER_POSITION_OPEN);
    }

    // Closes the claw down to hold the cone (it is not all the way closed)
    public void grabberHold() {
        grabber.setPosition(GRABBER_POSITION_HOLD);
    }

    // Returns current position of the grabber. 0 is wide open (dropped cone)
    public double grabberPosition() {
        return grabber.getPosition();
    }

}
