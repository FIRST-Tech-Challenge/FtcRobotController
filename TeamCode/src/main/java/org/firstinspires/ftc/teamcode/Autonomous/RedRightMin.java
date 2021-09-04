package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "RedRightMin")
public class RedRightMin extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, true);
//        robot.closeWobbleGoalClaw();
        sleep(1000);
//        robot.setPosition(0,0,14);
//        int rings = robot.getRingsAndWaitForStart();
//        robot.stopRingDetection();
//        robot.setPosition(0,0,14);
//        robot.track();
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
//        robot.setPosition(
//                0,0,14);
//        robot.goToPosition(-20,-5,-7,0.5);
//        robot.setVelocity(1550, 1000);
//        sleep(1000);
//        for (int i = 0; i < 3; i++) {
//            if(i>0) {
//                sleep(130);
//            }
//            robot.moveServo(false);
//            robot.moveServo(true);
//        }
//        robot.setVelocity(0, 1000);
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
//        if (rings == 0) {
//            robot.goToPosition(-55, -5, 0, 0.5);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
//            sleep(500);
//            robot.openWobbleGoalClaw();
//            sleep(500);
//            robot.moveWobbleGoalToPosition((WobbleGoal.Position.RAISE));
//            sleep(500);
//            robot.closeWobbleGoalClaw();
//            sleep(500);
//            robot.goToPosition(-37, -5, 0, 0.5);
//            robot.goToPosition(-37, 38, 0, 0.5);
//            robot.goToPosition(-68, 38, 0, 0.5);
//        }
//        else if (rings == 1) {
//            robot.turnInPlace(-90, 0.5);
//            robot.goToPosition(-91, -4, -90, 0.5);
//            robot.goToPosition(-119, -4, -90, 0.5);
//            robot.goToPosition(-119, 42, -90, 0.5);
//            robot.goToPosition(-80, 42, -90, 0.5);
//        }
        stop();
    }
}
