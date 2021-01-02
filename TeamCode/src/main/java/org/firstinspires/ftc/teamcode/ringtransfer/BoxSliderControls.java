package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxSliderControls {

    public Servo slider = null;

    private double _slideOut = 0.25;
    private double _slideIn = 1.0;
    public boolean _sliderIn = true;
    private boolean _b = false;

    public void initialize(LinearOpMode op) {
        slider = op.hardwareMap.get(Servo.class, "SlideServo");
        slider.setPosition(_slideIn);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.b && !_b) {
            _sliderIn =! _sliderIn;
        }
        _b = gamepad.b;
    }

    public boolean whileOpModeIsActive (LinearOpMode op, boolean _tiltingDown) {
        this.readController(op.gamepad2);
        /*if (_sliderIn == true) {
            slider.setPosition(_slideIn);
        } else {
            slider.setPosition(_slideOut);
        } */

        if ((_sliderIn == true) && (_tiltingDown == true)) {
            slider.setPosition(_slideIn);
        } else if ((_sliderIn == true) && (_tiltingDown == false)) {
            slider.setPosition(_slideOut);
            _sliderIn = false;
        } else {
            slider.setPosition(_slideOut);
        }

        return _sliderIn;
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}