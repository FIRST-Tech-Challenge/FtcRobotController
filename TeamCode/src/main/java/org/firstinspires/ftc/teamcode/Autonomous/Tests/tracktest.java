package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name= "tracktest ", group="Tests: ")
public class tracktest extends LinearOpMode{
final boolean debug = true;

    @Override
    public void runOpMode(){

        //Robot robot=new Robot(this, BasicChassis.ChassisType.ODOMETRY, false ,false);
        OdometryChassis robot = new OdometryChassis(this,false,true);
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        sleep(500);
        waitForStart();
        //robot.navigate();
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
        /*robot.moveAngle(0,72,0.5);
        sleep(5000);
        robot.moveAngle(0,-72,0.5);
        sleep(5000);*/
//        robot.goToPosition(48,0,0,0.8);
//        robot.goToPosition(0,0,0,0.8);//0.75,7/8,1/2,7/8,1/2,5/8,9/8,1/8,7/8
//        sleep(5000);
//        robot.moveAngle(48,48,0.5);
//        sleep(5000);
//        robot.moveAngle(-48,-48,0.5);
           //robot.moveRight(48,0.5);
           //robot.moveLeft(48,0.5);
        //robot.moveForward(24,0.5);
        //robot.moveBackward(24,0.5);
        //}
        robot.goToPosition(72,0,0,0.5);
        sleep(3000);
        robot.goToPosition(0,0,0,0.5);
        sleep(3000);
        robot.goToPosition(0,48,0,0.5);
        sleep(3000);
        robot.goToPosition(0,0,0,0.5);
        sleep(3000);
        robot.goToPosition(48,48,-90,0.5);
        sleep(3000);
        robot.goToPosition(0,0,0,0.5);
        sleep(3000);
        stop();
    }



}

