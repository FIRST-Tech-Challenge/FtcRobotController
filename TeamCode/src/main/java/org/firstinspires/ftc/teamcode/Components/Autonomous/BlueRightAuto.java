package org.firstinspires.ftc.teamcode.Components.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@Config
@Autonomous(name = "BlueRightAuto")


public class BlueRightAuto extends LinearOpMode {
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);

        //detectSignal();

        waitForStart();


    }
}
