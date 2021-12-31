package org.firstinspires.ftc.blackSwan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackSwan.Robot;

@Autonomous

public class AutonomousRedWarehouse extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.forward(1.15,.4);
        //This is the color Sensor Code (supposedly)
//        while(true){
//            telemetry.addData("Right green ", robot.colorSensorRight.green());
//            //telemetry.addData("Left green ", colorSensorLeft.red());
//            telemetry.update();
//        }
        if (robot.colorSensorRight.green() > 100) {
            //when it's dancing, you're supposed to pick it up
            robot.back(0.75,0.25);
            robot.turnRight(359, 0.5);
        } else {
            robot.right(0.85, .4);
            if (robot.colorSensorRight.green() > 100) {
                //when it's dancing, you're supposed to pick it up
                robot.back(0.78, 0.25);
                robot.turnRight(359,0.5);
            } else {
                //this will be when neither have the element, so turn and pick it up
                robot.back(0.5,0.5);
            }
        };


      /*  robot.left(1.75,.5);
        robot.back(.25,.5);
        robot.armThing(3);
        robot.turnRight(70,.5);
        robot.forward(6,.9);
        */
    }
}
