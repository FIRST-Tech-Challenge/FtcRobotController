package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;

public class Robot {
    OpMode _Opmode;
    //constructor
    public Robot(OpMode opmode){
        _Opmode = opmode;
    }
    //-------------------------------------------------------------------------------

    //Creates instance of MechanismControllers
    Viper _Viper = new Viper(_Opmode);
    WristClaw _WristClaw = new WristClaw(_Opmode);
    Arm _Arm = new Arm(_Opmode, new TelemetryHelper(_Opmode));
}
