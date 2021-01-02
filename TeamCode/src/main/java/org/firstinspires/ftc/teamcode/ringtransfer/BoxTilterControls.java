package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxTilterControls {

    public Servo tilter = null;

    private double _tiltUp = 0.81;
    private double _tiltDown = 0.0;
    public boolean _tiltingDown = true;
    private boolean _lftDpad = false;

    public void initialize(LinearOpMode op) {
        tilter = op.hardwareMap.get(Servo.class, "TiltServo");
        tilter.setPosition(_tiltDown);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.dpad_left && !_lftDpad) {
            _tiltingDown =! _tiltingDown;
        }
        _lftDpad = gamepad.dpad_left;
    }

    public boolean whileOpModeIsActive (LinearOpMode op, boolean _sliderIn) {
        this.readController(op.gamepad2);
        if (_tiltingDown == true) {
            tilter.setPosition(_tiltDown);
        } else {
            tilter.setPosition(_tiltUp);
        }

        if ((_tiltingDown == false) && (_sliderIn == false)) {
            tilter.setPosition(_tiltUp);
        } else if ((_tiltingDown == false) && (_sliderIn == true)) {
            tilter.setPosition(_tiltDown);
            _tiltingDown = true;
        } else {
            tilter.setPosition(_tiltDown);
        }

        return _tiltingDown;
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}