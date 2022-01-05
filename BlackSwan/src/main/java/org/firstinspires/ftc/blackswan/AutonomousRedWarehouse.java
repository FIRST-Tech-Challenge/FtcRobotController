package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousRedWarehouse extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.forward(1.15, .4);
        //This is the color Sensor Code (supposedly)
//        while (true) {
        telemetry.addData("Right green ", robot.colorSensorRight.green());
        //telemetry.addData("Left green ", colorSensorLeft.red());
        telemetry.update();
        // }
        if (robot.colorSensorRight.green() > 100) {
            robot.armThing(1);
        } else {
            robot.right(0.85, .4);
            if (robot.colorSensorRight.green() > 100) {
//                //this would mean that the block needs to be on the middle level
//                //we need to put movement here (to get to the shipping hub)
                robot.armThing(2);
            } else {
//                //this would mean that the block needs to be on the top level
//                //we need to put movement here (to get to the shipping hub)
                robot.armThing(3);
            }
        }


        robot.left(1.75, .5);
        robot.back(.25, .5);
        robot.armThing(3);
        robot.turnRight(70, .5);
        robot.forward(6, .9);
    }

  }

