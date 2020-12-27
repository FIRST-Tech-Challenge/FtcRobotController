package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TimerControls {

    ElapsedTime Timer;
    double _time = 0.0;
    double _lastTime = 0.0;

    public void initialize(LinearOpMode op) {
        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void startControl() {
        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();
    }

    public void resetTimer() {
        Timer.reset();
    }

    public double whileOpModeIsActive (LinearOpMode op) {
        _time = Timer.milliseconds();
        return _time;
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Timer (ms)", _time);
    }

    public void stop () {

    }
}