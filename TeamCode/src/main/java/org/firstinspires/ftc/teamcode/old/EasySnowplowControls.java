package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EasySnowplowControls {

    public Servo lftPlow = null;
    public Servo rgtPlow = null;

    private double _lftPosClosed = 0.0;
    private double _rgtPosClosed = 1.0;
    private double _lftPosOpen = 1.0;
    private double _rgtPosOpen = 0.0;
    private boolean _lftOpen = false;
    private boolean _rgtOpen = false;
    private boolean _x = false;
    private boolean _b = false;

    public void initialize(LinearOpMode op) {
        lftPlow = op.hardwareMap.get(Servo.class, "LftPlow");
        rgtPlow = op.hardwareMap.get(Servo.class, "RgtPlow");

        lftPlow.setPosition(_lftPosClosed);
        rgtPlow.setPosition(_rgtPosClosed);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.x && !_x) {
            _lftOpen = !_lftOpen;
        }
        _x = gamepad.x;

        if (gamepad.b && !_b) {
            _rgtOpen = !_rgtOpen;
        }
        _b = gamepad.b;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        if (_lftOpen == true) {
            lftPlow.setPosition(_lftPosOpen);
        } else {
            lftPlow.setPosition(_lftPosClosed);
        }
        if (_rgtOpen == true) {
            rgtPlow.setPosition(_rgtPosOpen);
        } else {
            rgtPlow.setPosition(_rgtPosClosed);
        }
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}