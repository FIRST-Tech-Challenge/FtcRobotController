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
    public final double verticalPos = Settings.Hardware.Servo.Linkage.VERTICAL_POSITION;
    public final double chamberPos = Settings.Hardware.Servo.Linkage.HIGH_CHAMBER_POSITION;
    public final double basketPos = Settings.Hardware.Servo.Linkage.HIGH_BASKET_POSITION;

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
            case VERTICAL:
                position = verticalPos;
                break;
            case BASKET:
                position = basketPos;
                break;
            case CHAMBER:
                position = chamberPos;
                break;
            case TRANSFER:
            default:
                position = transferPos;
                break;
        }
        linkageServo.setPosition(position);
    }

    public Position position() {
        if (position == verticalPos) {
            return Position.VERTICAL;
        } else if (position == basketPos) {
            return Position.BASKET;
        } else if (position == chamberPos) {
            return Position.CHAMBER;
        } else if (position == transferPos) {
            return Position.TRANSFER;
        } else {
            return Position.UNKNOWN;
        }
    }

    public void cyclePosition() {
        Position currentPosition = position();
        Position nextPosition;

        switch (currentPosition) {
            case VERTICAL:
                nextPosition = Position.CHAMBER;
                break;
            case CHAMBER:
                nextPosition = Position.BASKET;
                break;
            case TRANSFER:
            default:
                nextPosition = Position.VERTICAL;
                break;
        }

        setPosition(nextPosition);
    }

    public enum Position {
        TRANSFER,
        VERTICAL,
        CHAMBER,
        BASKET,
        UNKNOWN,
    }

}
