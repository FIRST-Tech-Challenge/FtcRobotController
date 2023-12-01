package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class WristSubsystem {
    private Servo wrist;
    private double wristPosition;
    private ElapsedTime runtime;
    Telemetry telemetry;

    public enum Positions {
        DOWN,
        UP
    }

    public WristSubsystem(Servo wrist, ElapsedTime runtime, Telemetry telemetry) {
        this.wrist = wrist;
        this.runtime = runtime;
        this.telemetry = telemetry;
        initialize();
    }

    public void initialize() {
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

    public double getWristPosition() {
        return wrist.getPosition();
    }
}
