package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {
    private final double OPEN = 1;
    private final double CLOSE = 0;
    private double currentPos = 0;
    private Servo wrist = null;
    private Telemetry telemetry;
    private boolean IS_DEBUG = false;

    public Wrist(OpMode opMode, boolean IS_DEBUG) {
        this.wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        this.telemetry = opMode.telemetry;
        this.IS_DEBUG = IS_DEBUG;
    }

    public Wrist(OpMode opMode) {
        new Wrist(opMode, false);
    }

    public void samples() {
        wrist.setPosition(OPEN);

        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Samples");
        }

    }

    public void specimen() {
        wrist.setPosition(CLOSE);

        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Specimen");
        }
    }

    public void changeState() {
        if (currentPos == OPEN) {
            wrist.setPosition(CLOSE);
            currentPos = CLOSE;
        } else {
            wrist.setPosition(OPEN);
            currentPos = OPEN;
        }

        if (IS_DEBUG) {
            telemetry.addData("Wrist", "Change State");
            telemetry.addData("Current Pos", currentPos);
        }
    }

    public double getCurrentPos() {
        return currentPos;
    }

    public void setCurrentPos(double currentPos) {
        this.currentPos = currentPos;
    }

    public Servo getWrist() {
        return wrist;
    }

    public void setWrist(Servo wrist) {
        this.wrist = wrist;
    }

    public double getOPEN() {
        return OPEN;
    }

    public double getCLOSE() {
        return CLOSE;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }
}
