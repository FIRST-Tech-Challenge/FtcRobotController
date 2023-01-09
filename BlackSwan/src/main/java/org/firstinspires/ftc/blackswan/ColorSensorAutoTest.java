package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class ColorSensorAutoTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        if (robot.colorSensorL.red() == 123){
            robot.left(.5, .3);
        }
        if (robot.colorSensorR.red() == 123) {
            robot.right(.5, .3);
        }
    }
}