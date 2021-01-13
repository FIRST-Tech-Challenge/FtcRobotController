package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;



@Autonomous(name= "track")
public class track extends LinearOpMode{


    @Override
    public void runOpMode(){

        //Robot robot=new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        OdometryChassis odom = new OdometryChassis(this);

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        while(isStopRequested()==false){
            odom.track();
        }
        stop();
    }



}

