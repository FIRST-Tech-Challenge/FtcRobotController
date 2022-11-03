package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class LeftAuton extends LinearOpMode
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
        waitForStart();
        int finalID = init.stopAndSave();
        telemetry.addLine(Integer.toString(finalID));
        telemetry.update();
        Auton auton = new Auton(true, finalID);
        auton.runAutonMain(new SampleMecanumDrive(hardwareMap), hardwareMap);
    }
}