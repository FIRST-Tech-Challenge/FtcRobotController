package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Wrist {
    public static double position = 0;
    public final Servo wristLeft;
    public final Servo wristRight;
    public final double verticalPos = Settings.Hardware.Servo.Wrist.VERTICAL_POSITION;
    public final double chamberPos = Settings.Hardware.Servo.Wrist.CHAMBER_POSITION;
    public final double horizPos = Settings.Hardware.Servo.Wrist.HORIZONTAL_POSITION;

    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;

    public Wrist(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        wristLeft = hardwareMap.get(Servo.class, Settings.Hardware.IDs.WRIST_LEFT);
        wristRight = hardwareMap.get(Servo.class, Settings.Hardware.IDs.WRIST_RIGHT);
        wristRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(Position newPosition) {
        double oldPosition = position;
        switch (newPosition) {
            case VERTICAL:
                position = verticalPos;
                break;
            case CHAMBER:
                position = chamberPos;
                break;
            default:
                position = horizPos;
                break;
        }
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
    }

    public Position position() {
        if (position == verticalPos) {
            return Position.VERTICAL;
        } else if (position == chamberPos) {
            return Position.CHAMBER;
        } else if (position == horizPos) {
            return Position.HORIZONTAL;
        } else {
            return Position.UNKNOWN;
        }
    }

    public void cyclePosition() {
        Position currentPosition = position();
        Position nextPosition;

        switch (currentPosition) {
            case HORIZONTAL:
                nextPosition = Position.CHAMBER;
                break;
            case VERTICAL:
                nextPosition = Position.HORIZONTAL;
                break;
            case CHAMBER:
                nextPosition = Position.VERTICAL;
                break;
            default:
                nextPosition = Position.HORIZONTAL; // Fallback to HORIZONTAL if unknown
                break;
        }

        setPosition(nextPosition);
    }

    public enum Position {
        HORIZONTAL,
        VERTICAL,
        CHAMBER,
        UNKNOWN,
    }

}
