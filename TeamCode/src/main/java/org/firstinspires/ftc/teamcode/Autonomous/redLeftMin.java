package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "RedLeftMin", preselectTeleOp = "OneGPTeleop")
public class redLeftMin extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, true);
        robot.setPosition(0,0,-36);
        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
        robot.setPosition(0,0,-36);
        robot.track();
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
        robot.setPosition(0,0,-36);
        robot.goToPosition(-20,5,14,0.5);
        robot.setVelocity(1500, 1000);
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
            robot.goToPosition(-48, -48, 0, 0.5);
//            robot.goToPosition(-37, -5, 0, 0.5);
//            robot.goToPosition(-37, 38, 0, 0.5);
//            robot.goToPosition(-68, 38, 0, 0.5);
        }
//        else if (rings == 1) {
//            robot.goToPosition(-91, -5, -90, 0.5);
//            robot.goToPosition(-135, -5, 0, 0.5);
//            robot.goToPosition(-135, 50, 0, 0.5);
//            robot.goToPosition(-79, 50, 0, 0.5);
//        }
        stop();
    }
}
