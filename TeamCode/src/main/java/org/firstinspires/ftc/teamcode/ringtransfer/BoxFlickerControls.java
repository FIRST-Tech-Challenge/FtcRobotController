package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxFlickerControls {

    public CRServo flicker = null;
    public DigitalChannel magneticSwitch;

    private boolean _flickerKicking = false;
    private boolean _startButton = false;
    private int _switchDetected = 0;

    public void initialize(LinearOpMode op) {
        flicker = op.hardwareMap.get(CRServo.class, "ShooterFlick");
        magneticSwitch = op.hardwareMap.get(DigitalChannel.class, "FlickMagnet");
        magneticSwitch.setMode(DigitalChannel.Mode.INPUT);

        flicker.setPower(0.0);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        /* if (gamepad.start && !_startButton) {
            _flickerKicking =! _flickerKicking;
        }

        _startButton = gamepad.start; */

        if (gamepad.start) {
            _flickerKicking = true;
        } else {
            _flickerKicking = false;
        }
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        /* if ((_flickerKicking == true) && (_switchDetected < 3)) {
            flicker.setPower(-1.0);
            if (magneticSwitch.getState() == true) {
                _switchDetected = _switchDetected + 1;
            }
        } else {
            flicker.setPower(0.0);
            _switchDetected = 0;
            _flickerKicking = false;
        } */

        if (_flickerKicking == true) {
            flicker.setPower(-1.0);
            if (magneticSwitch.getState() == true) {
                _switchDetected = _switchDetected + 1;
            }
        } else {
            flicker.setPower(0.0);
            //_switchDetected = 0;
            _flickerKicking = false;
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Flicker Kicking", _flickerKicking);
        telemetry.addData("Magnetic Touch", _switchDetected);
    }

    public void stop () {
        flicker.setPower(0.0);
    }
}