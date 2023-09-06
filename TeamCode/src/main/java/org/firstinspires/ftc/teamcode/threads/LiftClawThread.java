package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.subsystems.hardware.DistanceSensorDevice;
import org.firstinspires.ftc.teamcode.subsystems.hardware.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Lights;

public class LiftClawThread extends RobotThread {
    private final Gamepad _gamepad;


    LiftClaw _claw;

    Lights _light;

    public LiftClawThread(DcMotor Motor, Servo [] servos, TouchSensor stop, DistanceSensorDevice release,
                          Telemetry telemetry, Gamepad gamepad, Lights light) {
        _gamepad=gamepad;
        _claw = new LiftClaw(Motor,servos,stop,release,telemetry,light);
        _light = light;
    }

    public void run() {
        _claw.calibrateLift();
        while (!isCancelled()) {
            //lift movement
            if (_claw.checkOptical()) {
                _light.blinkOnce(300);
                _claw.setClawOpenTime(System.currentTimeMillis());
                _claw.clawOpen();
            }

            _claw.moveLift(_gamepad.left_stick_y);
            //  claw control
            if (_gamepad.right_trigger > 0) {
                _claw.clawOpen();
            } else {
                _claw.clawClose();
            }
            if (_gamepad.x) {
                _claw.runToPos(LiftClaw.BOTTOM_POS);
            }
            if (_gamepad.a) {
                _claw.runToPos(LiftClaw.LOW_POS);
            }
            if (_gamepad.b) {
                _claw.runToPos(LiftClaw.MEDIUM_POS);
            }
            if (_gamepad.y) {
                _claw.runToPos(LiftClaw.HIGH_POS);
            }
            if (Math.abs(_gamepad.left_stick_y) > 0.1) {
                _claw.triggerStop();
            }
        }
    }
}
