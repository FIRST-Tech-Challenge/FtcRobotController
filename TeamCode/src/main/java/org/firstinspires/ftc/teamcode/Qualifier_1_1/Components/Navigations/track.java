package org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.Navigations;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.EncoderChassis;


@Autonomous(name= "track")
public class track extends LinearOpMode{


    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER);
        EncoderChassis odom = new EncoderChassis(this);

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

