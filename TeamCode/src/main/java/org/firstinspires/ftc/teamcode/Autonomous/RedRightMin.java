package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "RedRightMin", preselectTeleOp = "OneGPTeleop")
public class RedRightMin extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, true);
        robot.setPosition(0,0,14);
        /*int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
        robot.setPosition(0,0,14);
        robot.track();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
        robot.setPosition(0,0,14);
        robot.goToPosition(-20,-5,-7,0.5);
        robot.setVelocity(1550, 1000);
        sleep(1000);
        for (int i = 0; i < 3; i++) {
            if(i>0) {
                sleep(130);
            }
            robot.moveServo(false);
            robot.moveServo(true);
        }
        robot.setVelocity(0, 1000);
        if (rings == 0) {
            robot.goToPosition(-55, -5, 0, 0.5);
            robot.goToPosition(-37, -5, 0, 0.5);
            robot.goToPosition(-37, 38, 0, 0.5);
            robot.goToPosition(-68, 38, 0, 0.5);
        }*/
        waitForStart();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
        stop();
    }
}
