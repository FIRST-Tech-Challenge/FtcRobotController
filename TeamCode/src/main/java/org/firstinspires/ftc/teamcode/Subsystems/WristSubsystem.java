package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class WristSubsystem {
    private Servo wristHardwareMap, wrist;
    private double wristPosition;
    private ElapsedTime runtime;

    public WristSubsystem(Servo wristHardwareMap, ElapsedTime runtime) {
        this.wristHardwareMap = wristHardwareMap;
        this.runtime = runtime;
    }

    public void initialize() {
        wrist = wristHardwareMap;
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.scaleRange(Constants.wristMinPosition, Constants.wristMaxPosition);
        wristPosition = 0.0;
    }

    public void manualAngleWrist(boolean gamepad2RightBumper, boolean gamepad2LeftBumper) {
        if (gamepad2RightBumper) {
            wristPosition += Constants.teleOPWristIncrement;
            Range.clip(wristPosition, Constants.wristMinPosition, Constants.wristMaxPosition);
        }
        else if (gamepad2LeftBumper) {
            wristPosition -= Constants.teleOPWristIncrement;
            Range.clip(wristPosition, Constants.wristMinPosition, Constants.wristMaxPosition);
        }

        wrist.setPosition(wristPosition);
    }

    public void autoAngleWrist(Positions position) {
        switch (position) {
            case UP:
                wristPosition = Constants.wristMinPosition;
                break;

            case DOWN:
                wristPosition = Constants.wristMaxPosition;
                break;

            default:
                break;
        }

        wrist.setPosition(wristPosition);
    }

    public enum Positions {
        DOWN,
        UP
    }
}
