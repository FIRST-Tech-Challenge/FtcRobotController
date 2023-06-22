package org.firstinspires.ftc.teamcode.components;

// TODO: 21/01/2023 Test

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

/**
 * Team 2's lift which lifts to a certain height
 */
public class GrabberComponent {
    public enum GrabberPosition {
        CLOSED(0.55),
        OPEN(0.0);
        private final double position;

        GrabberPosition(double position) {
            this.position = position;
        }

        @Override
        public String toString() {
            return "GrabberPosition{" +
                    "position=" + position +
                    '}';
        }
    }

    // Degrees
    private static final double MAX_SERVO_ANGLE = 40;
    private static final double MIN_SERVO_ANGLE = 0;

    ServoEx left_intake;
    SimpleServo right_intake;

    public GrabberPosition position;

    /**
     * Create a {@code LiftComponent}
     */
    public GrabberComponent(int servo1_id, int servo2_id) {
        left_intake = HardwareMapContainer.getServo(servo1_id, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, false, AngleUnit.DEGREES);
        right_intake = HardwareMapContainer.getServo(servo2_id, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, true, AngleUnit.DEGREES);
    }

    /**
     * Sets the fraction of how much the intake is opened, from 0 to 1
     */
    public void setPosition(GrabberPosition position) {
        this.position = position;
        double numericalPosition = position.position;
        left_intake.setPosition(numericalPosition);
        right_intake.setPosition(numericalPosition);
    }

    public void togglePosition(){
        GrabberPosition newPosition = this.position == GrabberPosition.CLOSED ? GrabberPosition.OPEN : GrabberPosition.CLOSED;
        setPosition(newPosition);
    }
}
