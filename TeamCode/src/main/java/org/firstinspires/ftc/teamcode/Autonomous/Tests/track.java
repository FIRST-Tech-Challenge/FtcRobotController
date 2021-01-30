package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "tack")
public class track extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        OdometryChassis odom = new OdometryChassis(this);
        ElapsedTime op = new ElapsedTime();

        //ElapsedTime runtime = new ElapsedTime();

        int rings = 4;//robot.getRingsAndWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        //robot.stopRingDetection();
        //odom.setPosition(-15.75,61.75, 0);
        telemetry.update();

        waitForStart();
        op.startTime();
        while(op.seconds()<3){
            odom.track();
        }
        odom.goToPosition(0,0,0, 0.8);
        sleep(2000);

        stop();
    }



}
