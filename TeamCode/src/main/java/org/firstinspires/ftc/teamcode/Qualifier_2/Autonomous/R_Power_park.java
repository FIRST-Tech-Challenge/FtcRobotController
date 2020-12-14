package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;

@Autonomous(name= "R_Power_park")
public class R_Power_park extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.runTensorFlowWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();
        telemetry.update();
        //waitForStart();
        if(rings==0) {
            robot.moveAngle( -53,-55, 0.6);
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,8,0.8);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(37,2,0.5);
            robot.turnInPlace(2.0,1.0);
        }
        else if(rings==1) {
            robot.moveAngle(4,-83, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,4.5, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(10,22, 0.7);
            robot.turnInPlace(2,1.0);
        }
        else if(rings==4) {
            robot.moveAngle(-48, -102,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(-0, 12,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(2,40.5,0.7);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(39,2,0.7);
            robot.turnInPlace(2,1.0);
        }
        robot.shootRightPowerShot(3);

        sleep(500);
        stop();
    }




}
