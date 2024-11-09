package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Wrist {
    public static double position = 0;
    public final Servo wristServo;
    public final double verticalPos = Settings.Hardware.Servo.Wrist.VERTICAL_POSITION;
    public final double chamberPos = Settings.Hardware.Servo.Wrist.CHAMBER_POSITION;
    public final double basketPos = Settings.Hardware.Servo.Wrist.BASKET_POSITION;
    public final double horizPos = Settings.Hardware.Servo.Wrist.HORIZONTAL_POSITION;

    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    private final String LOG_PREFIX = "Wrist: ";

    public Wrist(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        wristServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.WRIST);
        setPosition(Position.HORIZONTAL);
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
            case BASKET:
                position = basketPos;
                break;
            default:
                position = horizPos;
                break;
        }
        wristServo.setPosition(position);
        baseRobot.logger.update(LOG_PREFIX + "Position",
                String.format("%s (%.3f → %.3f)", newPosition, oldPosition, position));
    }

    public Position position() {
        if (position == verticalPos) {
            return Position.VERTICAL;
        } else if (position == chamberPos) {
            return Position.CHAMBER;
        } else if (position == basketPos) {
            return Position.BASKET;
        } else if (position == horizPos) {
            return Position.HORIZONTAL;
        } else {
            return Position.UNKNOWN;
        }
    }

    public enum Position {
        HORIZONTAL,
        VERTICAL,
        CHAMBER,
        BASKET,
        UNKNOWN,
    }

}
