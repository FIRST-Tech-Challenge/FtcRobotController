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
        robot.forward(1.5, .2);
        //This is the color Sensor Code (supposedly)
//        while (true) {
            telemetry.addData("Right green ", robot.colorSensorRight.green());
            //telemetry.addData("Left green ", colorSensorLeft.red());
            telemetry.update();
            // }
            if (robot.colorSensorRight.green() > 100) {
                robot.back(0.15, 0.5);
                robot.left(1, 0.5);
                //robot.forward();
                robot.armThing(1);
            } else {
                robot.right(0.85, .2);
                if (robot.colorSensorRight.green() > 100) {
                    //this would mean that the block needs to be on the middle level
                    robot.back(0.15, 0.5);
                    robot.left(2, 0.5);
                    //robot.forward();
                    robot.armThing(2);
                } else {
                    //this would mean that the block needs to be on the top level
                    robot.back(0.15, 0.5);
                    robot.left(2, 0.5);
                    //robot.forward();
                    robot.armThing(3);
                }
            }
            //this is the code for parking/going to another block
//        robot.back(.25, .5);
//        robot.turnRight(70, .5);
//        robot.forward(6, .9);

            //the code for picking up a new shipping element
//            robot.turnRight(30,0.5);
//            robot.intake.setPower(-1);
//            robot.forward(0.2,0.2);
//            robot.intake.setPower(0);
//            robot.back(0.2,0.2);
//            robot.turnLeft(30,0.5);
//            robot.forward(6,.5);
       // }
    }

  }

