package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Baby's First Auto")
public class babysFirstAuto extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        robot.init();
        robot.forward(.3, .5);

    }

}
