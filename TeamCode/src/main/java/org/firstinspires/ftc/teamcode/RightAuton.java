package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class RightAuton extends LinearOpMode
{

    @Override
    public void runOpMode()
    {

        aprilTagsInit init = new aprilTagsInit(hardwareMap,telemetry);
        init.initialize();
        while(!isStarted() && !isStopRequested())
        {
            init.search();
            sleep(20);
        }
        int finalID = init.stopAndSave();
        Auton auton = new Auton(false, finalID);
        auton.runAuton(new Robot(hardwareMap));
    }

}