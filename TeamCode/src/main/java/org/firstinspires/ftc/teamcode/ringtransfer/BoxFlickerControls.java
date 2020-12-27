package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxFlickerControls {

    public Servo flicker = null;

    private double _flickerPosNotKick = 0.4;
    private double _flickerPosKick = 0.05;
    private boolean _flickerKicking = true;
    private boolean _y = false;

    public void initialize(LinearOpMode op) {
        flicker = op.hardwareMap.get(Servo.class, "ShooterFlick");

        flicker.setPosition(_flickerPosKick);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.y && !_y) {
            _flickerKicking =! _flickerKicking;
        }
        _y = gamepad.y;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        if (_flickerKicking == true) {
            flicker.setPosition(_flickerPosKick);
        } else {
            flicker.setPosition(_flickerPosNotKick);
        }
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}