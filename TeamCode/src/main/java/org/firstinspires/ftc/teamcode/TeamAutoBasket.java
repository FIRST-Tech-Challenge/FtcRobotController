package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Basket", group = "Autonomous")
//@Disabled
public class TeamAutoBasket extends LinearOpMode {
    //private final double robot_power = 1.0;
    BBRobot robot;

//    TeamAutoBlueBasket(){
//        robot = new BBRobot(hardwareMap, telemetry);
//        initRobotSettings();
//    }

    private void initRobotSettings() {
//        robot.turnSlide(robot.robot_power,0, 1500,false);
        robot.wrist_end();
        robot.clawClose();
    }

    @Override
    public void runOpMode()
    {
//        new TeamAutoBlueBasket();
        robot = new BBRobot(hardwareMap, telemetry);
        initRobotSettings();
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //move towards the center
            robot.moveBackwardToPosition(robot.robot_power, 34,3400);

            //hang element on the bard
            robot.hangElementOnHighBar(robot.robot_power);
//
//            int turn_deg = (int) robot.getAngle();
////            robot.rotateAntiClock(turn_deg, robot_power);
//            telemetry.addData("Current agnel is", "%5d", turn_deg);
//            telemetry.update();

            // go to first element
            robot.rotateAntiClock(-90, robot.robot_power);
            robot.moveForwardToPosition(robot.robot_power, 48, 4500);
            robot.rotateAntiClock(-90, robot.robot_power);
            // pick up and drop pixel7
//            robot.turnSlideBack();
            robot.moveForwardToPosition(robot.robot_power, 1.5, 500);
            robot.elementGrab();
//            robot.turnSlideForDrop();

            robot.moveBackwardToPosition(robot.robot_power, 15,  2400);
            robot.rotateAntiClock(-45, robot.robot_power);
            robot.expandSlide();
//            robot.wrist_end();
//            sleep(100);
            robot.dropElement();
//            sleep(100);
//            robot.contractSlide();

            // reset the slide to original position
//            robot.turnSlideUp();
            robot.rotateAntiClock(45, robot.robot_power);
            robot.moveForwardToPosition(robot.robot_power, 52,  4000);
            robot.rotateAntiClock(45, robot.robot_power);
            robot.rotateAntiClock(45, robot.robot_power);
//            robot.rotateAntiClock(150, robot.robot_power);
            robot.moveBackwardToPosition(robot.robot_power, 25,  2400);
            break;
        }
    }

}
