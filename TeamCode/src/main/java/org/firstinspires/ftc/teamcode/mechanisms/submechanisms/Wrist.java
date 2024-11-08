package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Wrist {
    public static double position = 0;
    public final Servo wristServo;
    public final double boardPos = Settings.Hardware.Servo.Wrist.BOARD_POSITION;
    public final double transitPos = Settings.Hardware.Servo.Wrist.TRANSIT_POSITION;
    public final double horizPos = Settings.Hardware.Servo.Wrist.HORIZONTAL_POSITION;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;

    public Wrist(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        wristServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.WRIST);
        setPosition(Position.HORIZONTAL);
    }

    public void setPosition(Position newPosition) {
        switch (newPosition) {
            case RUNG:
                position = boardPos;
                break;
            case NEUTRAL:
                position = transitPos;
                break;
            default:
                position = horizPos;
                break;
        }
        baseRobot.logger.update("wrist position", "0" + position);
        wristServo.setPosition(position);
    }

    public Position position() {
        if (position == boardPos) {
            return Position.RUNG;
        } else if (position == transitPos) {
            return Position.NEUTRAL;
        } else if (position == horizPos) {
            return Position.HORIZONTAL;
        } else {
            return Position.UNKNOWN;
        }
    }

    public enum Position {
        HORIZONTAL,
        RUNG,
        NEUTRAL,
        UNKNOWN,
    }

}
