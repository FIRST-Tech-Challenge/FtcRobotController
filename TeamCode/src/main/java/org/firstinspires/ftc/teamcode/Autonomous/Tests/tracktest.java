package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name= "tracktest ", group="Tests: ")
public class tracktest extends LinearOpMode{
final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.ODOMETRY, false ,false);
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        sleep(1000);
        waitForStart();
        /*robot.moveAngle(0,20,0.5);
        telemetry.addData("moved1s", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(0,-20,0.5);
        telemetry.addData("moved2s", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle( 20,0,0.5);
        telemetry.addData("moved3s", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle( -20,0,0.5);
        telemetry.addData("moved4s", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle( 15,15,0.5);
        telemetry.addData("moved5s", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle( -15,-15,0.5);
        telemetry.addData("moved6s", "done");
        telemetry.update();
        sleep(1000);*/
        /*robot.moveAngle(0,72,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(0,-72,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(24,0,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(-24,0,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(24,24,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);
        robot.moveAngle(-24,-24,0.5);
        telemetry.addData("moved", "done");
        telemetry.update();
        sleep(1000);*/
        //while(!isStopRequested()){
        robot.moveAngle(0,72,0.5);
        robot.moveAngle(0,-72,0.5);
        robot.moveAngle(48,0,0.5);
        robot.moveAngle(-48,0,0.5);
        robot.moveAngle(48,48,0.5);
        robot.moveAngle(-48,-48,0.5);
           //robot.moveRight(48,0.5);
           //robot.moveLeft(48,0.5);
        //robot.moveForward(24,0.5);
        //robot.moveBackward(24,0.5);
        //}
        stop();
    }



}

