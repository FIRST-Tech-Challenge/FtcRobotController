package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;

public class Wrist {
    public static double position = 0;
    public final Servo wristServo;
    public final double boardPos = 0.075;
    public final double transitPos = 0.5;
    public final double horizPos = 0.45;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;


    public Wrist(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        wristServo = hardwareMap.get(Servo.class, "wrist");
        setPosition(Position.HORIZONTAL);
    }

    public void setPosition(Position newPosition) {
        switch (newPosition) {
            case BOARD:
                position = boardPos;
                break;
            case TRANSIT:
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
            return Position.BOARD;
        } else if (position == transitPos) {
            return Position.TRANSIT;
        } else if (position == horizPos) {
            return Position.HORIZONTAL;
        } else {
            return Position.UNKNOWN;
        }
    }


    public enum Position {
        HORIZONTAL,
        BOARD,
        TRANSIT,
        UNKNOWN,
    }

}
