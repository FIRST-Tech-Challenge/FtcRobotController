package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;

import java.util.ArrayList;
import java.util.List;

public class Extender {
    private static final List<Double> positions = new ArrayList<>();
    private static int currentPositionIndex = 0;

    private final Servo extServo;
    private final BaseRobot baseRobot;

    /* A viper slide extender servo manager. */
    public Extender(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;

        // List of possible positions
        positions.add(0.0);  // ground position, not used much
        positions.add(0.15);  // 'hover' position just above the ground for movement
        positions.add(0.3);  // 'stackgrab' position to grab from white pixel stacks
        positions.add(0.66); // bar1
        positions.add(1.0); // bar2

        extServo = baseRobot.hardwareMap.get(Servo.class, "extenderServo");

        set(Position.HOVER);
    }

    /* Extends the extender to an exact position by index, used in autonomous. */
    public void set(int positionIndex) {
        currentPositionIndex = positionIndex;
        if (currentPositionIndex >= positions.size() - 1) {
            currentPositionIndex = positions.size() - 1;
        }
        if (currentPositionIndex < 0) {
            currentPositionIndex = 0;
        }
        extServo.setPosition(positions.get(currentPositionIndex));
    }

    public void set(Position positionIndex) {
        set(positionIndex.ordinal());
    }

    /* Extends the extender until retracted or stopped by those functions */
    public void extend() {
        currentPositionIndex = currentPositionIndex + 1;
        if (currentPositionIndex >= positions.size()) {
            currentPositionIndex = 0;
        }
        extServo.setPosition(positions.get(currentPositionIndex));
    }

    /* Brings the extender back in at the opposite speed */
    public void retract() {
        currentPositionIndex = currentPositionIndex - 1;
        if (currentPositionIndex < 0) {
            currentPositionIndex = positions.size() - 1;
        }
        extServo.setPosition(positions.get(currentPositionIndex));
    }

    /* Sets the extender position to 0 */
    public void ground() {
        currentPositionIndex = 0;
        extServo.setPosition(positions.get(currentPositionIndex));
    }

    public enum Position {
        GROUND,
        HOVER,
        STACKGRAB,
        LEVEL1,
        LEVEL2,
    }
}