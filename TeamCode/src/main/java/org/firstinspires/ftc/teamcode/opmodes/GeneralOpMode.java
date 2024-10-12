package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ITDbot;

abstract public class GeneralOpMode extends LinearOpMode {
    public ITDbot robot;
    
    public void initialize() {
        robot = new ITDbot();
        robot.Init(hardwareMap,telemetry);
    }
}
