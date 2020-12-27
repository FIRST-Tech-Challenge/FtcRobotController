package org.firstinspires.ftc.teamcode.Components.Navigations;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Robot;



//TODO: Warren, what is the purpose of this class and why is it here?
@Autonomous(name= "track")
public class track extends LinearOpMode{


    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER, false, false);
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

