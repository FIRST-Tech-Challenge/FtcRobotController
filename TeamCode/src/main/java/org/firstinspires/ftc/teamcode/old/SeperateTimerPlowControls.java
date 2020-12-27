package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SeperateTimerPlowControls {

    public Servo lftPlow = null;
    public Servo rgtPlow = null;

    private double _lftPosClosed = 0.0;
    private double _rgtPosClosed = 1.0;
    private double _lftPosOpen = 1.0;
    private double _rgtPosOpen = 0.0;
    private boolean _lftOpen = false;
    private boolean _rgtOpen = false;
    private boolean _x = false;
    private boolean _currentOpenPlows = false;
    private boolean _lastOpenPlows = false;

    ElapsedTime Timer;
    double _lastTime = 0.0;
    double time = 0.0;

    public void initialize(LinearOpMode op) {
        lftPlow = op.hardwareMap.get(Servo.class, "LftPlow");
        rgtPlow = op.hardwareMap.get(Servo.class, "RgtPlow");

        lftPlow.setPosition(_lftPosClosed);
        rgtPlow.setPosition(_rgtPosClosed);

        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void startControl() {
        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();
        openPlows(time);
        _lftOpen = true;
        _rgtOpen = true;
        _lastOpenPlows = true;
        _currentOpenPlows = true;
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.x && !_x) {
            _currentOpenPlows = !_currentOpenPlows;
        }
        _x = gamepad.x;
    }

    public void openPlows (double time) {
        rgtPlow.setPosition(_rgtPosOpen);
        _rgtOpen = true;
        if ((time - _lastTime) > 200) {
            lftPlow.setPosition(_lftPosOpen);
            _lftOpen = true;
            _lastTime = time;
            _lastOpenPlows = true;
        }
    }

    public void closePlows (double time) {
        lftPlow.setPosition(_lftPosClosed);
        _lftOpen = false;
        if ((time - _lastTime) > 200) {
            rgtPlow.setPosition(_rgtPosClosed);
            _rgtOpen = false;
            _lastTime = time;
            _lastOpenPlows = false;
        }
    }

    public void whileOpModeIsActive (LinearOpMode op, double time) {
        this.readController(op.gamepad2);

        if ((_currentOpenPlows == false) && (_lastOpenPlows == true)) {
            _lastTime = time;
            closePlows(time);
        }
        if ((_currentOpenPlows == true) && (_lastOpenPlows == false)) {
            Timer.reset();
            time = Timer.milliseconds();
            openPlows(time);
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Timer Snowplows",time);
    }

    public void stop () {
    }
}