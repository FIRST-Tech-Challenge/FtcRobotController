package org.firstinspires.ftc.teamcode.wobblegoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GripperControls {

    public Servo gripper = null;

    private double _gripPosOpen = 0.0;
    private double _gripPosClose = 0.65;
    private double _gripPosRing = 0.5;
    private double _gripPosCurrent = _gripPosClose;
    private boolean _flickerKicking = false;
    //private boolean _rgtBump = false;
    //private boolean _lftBump = false;
    //private boolean _a = false;

    public void initialize(LinearOpMode op) {
        gripper = op.hardwareMap.get(Servo.class, "Gripper");

        gripper.setPosition(_gripPosCurrent);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.a == true) {
            _gripPosCurrent = _gripPosRing;
        }
        //_a = gamepad.a;

        if (gamepad.right_bumper == true) {
            _gripPosCurrent = _gripPosOpen;
        }
        //_rgtBump = gamepad.right_bumper;

        if (gamepad.left_bumper == true) {
            _gripPosCurrent = _gripPosClose;
        }
        //_lftBump = gamepad.left_bumper;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        gripper.setPosition(_gripPosCurrent);
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}