package org.firstinspires.ftc.teamcode.lib.drivers;

import org.openftc.revextensions2.ExpansionHubServo;

public class RevServo {
    private static final double MIN_DELTA_POSITION = 0.01d;
    private ExpansionHubServo servo;
    private double lastPosition;

    public RevServo(ExpansionHubServo servo) {
        setServo(servo);
    }

    public void setPosition(double position) {
        //Ensure that the position argument is within the capabilities of the servo
        position = position < -1 ? -1 : position > 1 ? 1 : position;

        //Check if position is zero in the case of initialization, since the servo is defaulted
        //to zero position even though the servo is not powered by default.
        if(Math.abs(position - getLastPosition()) >= getMinDeltaPosition() || position == 0d) {
            getServo().setPosition(position);
            setLastPosition(position);
        }
    }

    public double getLastPosition() {
        return lastPosition;
    }

    public void setLastPosition(double lastPosition) {
        this.lastPosition = lastPosition;
    }

    public ExpansionHubServo getServo() {
        return servo;
    }

    public void setServo(ExpansionHubServo servo) {
        this.servo = servo;
    }

    public static double getMinDeltaPosition() {
        return MIN_DELTA_POSITION;
    }
}
