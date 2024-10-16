package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {
    private final double OPEN = 1;
    private final double CLOSE = 0;
    boolean dbug = false;
    private double currentPos = 0;
    private Servo wrist = null;
    private Telemetry telemetry;

    public Wrist(OpMode opMode, boolean dbug) {
        this.wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        this.telemetry = opMode.telemetry;
        this.dbug = dbug;
    }

    public Wrist(OpMode opMode) {
        new Wrist(opMode, false);
    }

    public void samples() {
        wrist.setPosition(OPEN);

        if (dbug) {
            telemetry.addData("Wrist", "Samples");
        }

    }

    public void specimen() {
        wrist.setPosition(CLOSE);

        if (dbug) {
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

        if (dbug) {
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
}
