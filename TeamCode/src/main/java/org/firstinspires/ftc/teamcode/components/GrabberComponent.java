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

    // Degrees
    private static double MAX_SERVO_ANGLE = 40;
    private static double MIN_SERVO_ANGLE = 0;

    ServoEx left_intake;
    SimpleServo right_intake;

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
    public void setFractionOpened(double f) {
        left_intake.setPosition(f);
        right_intake.setPosition(f);
    }
}
