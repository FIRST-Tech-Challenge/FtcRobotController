package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Linkage {
    public static double position = 0;
    public final Servo linkageServo;
    public final double transferPos = Settings.Hardware.Servo.Linkage.TRANSFER_POSITION;
    public final double highRungPos = Settings.Hardware.Servo.Linkage.HIGH_RUNG_POSITION;
    public final double placePos = Settings.Hardware.Servo.Linkage.PLACE_POSITION;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;

    public Linkage(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        linkageServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.LINKAGE);
    }

    public void setPosition(Position newPosition) {
        double oldPosition = position;
        switch (newPosition) {
            case PLACE:
                position = placePos;
                break;
            case HIGH_RUNG:
                position = highRungPos;
                break;
            case TRANSFER:
            default:
                position = transferPos;
                break;
        }
        linkageServo.setPosition(position);
    }

    public Position position() {
        if (position == placePos) {
            return Position.PLACE;
        } else if (position == transferPos) {
            return Position.TRANSFER;
        } else if (position == highRungPos){
            return Position.HIGH_RUNG;
        } else {
            return Position.UNKNOWN;
        }
    }

    public void cyclePosition() {
        Position currentPosition = position();
        Position nextPosition;

        switch (currentPosition) {
            case PLACE:
                nextPosition = Position.HIGH_RUNG;
                break;
            case HIGH_RUNG:
                nextPosition = Position.TRANSFER;
                break;
            case TRANSFER:
            default:
                nextPosition = Position.PLACE;
                break;
        }

        setPosition(nextPosition);
    }

    public enum Position {
        TRANSFER,
        PLACE,
        HIGH_RUNG,
        UNKNOWN,
    }

}
