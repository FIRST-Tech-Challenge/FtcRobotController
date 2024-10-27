package org.firstinspires.ftc.teamcode.BBcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WristClaw {
    OpMode _opMode;
    Servo _wrist;
    Servo _claw;
    public WristClaw (OpMode opMode)
    {
        _opMode = opMode;
        _wrist = _opMode.hardwareMap.tryGet(Servo.class, "wrist");
        _claw = _opMode.hardwareMap.tryGet(Servo.class, "claw");
    }
    //-----------------------------------------
    //Variable Storage:
    double openPosition = 0.2;
    double closePosition = 0.52;
    double upPosition = 1;
    double flipPosition = 0.45;
    double downPosition = 0.62;
    //-----------------------------------------

    public void OpenClaw() {ClawCustom(openPosition);}
    public void CloseClaw() {ClawCustom(closePosition);}
    public void MoveUp() {WristCustom(upPosition);}
    public void MoveFlip() {WristCustom(flipPosition);}
    public void MoveDown() {WristCustom(downPosition);}
    public void WristCustom(double position)
    {
        if (_wrist == null)
        {
            _opMode.telemetry.addLine("Wrist servo not found!");
        } else {
            _wrist.setPosition(position);
        }

    }
    public void ClawCustom(double position)
    {
        if (_claw == null)
        {
            _opMode.telemetry.addLine("Claw Servo not found!");
        } else {
            _claw.setPosition(position);
        }
    }
}
