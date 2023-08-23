package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.arch.core.internal.SafeIterableMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.teamcode.Controller.MechanicalDriveBase;

@Autonomous(name = "Sam thread test", group = "Sam")
public class ThreadMasterSam extends LinearOpMode
{
    ConceptTensorFlowObjectDetectionWebcamSam conceptTensorFlowObjectDetectionWebcamSam;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException
    {
        conceptTensorFlowObjectDetectionWebcamSam = new ConceptTensorFlowObjectDetectionWebcamSam();
        elapsedTime = new ElapsedTime();
        conceptTensorFlowObjectDetectionWebcamSam.initVuforia(hardwareMap);
        conceptTensorFlowObjectDetectionWebcamSam.initTfod(hardwareMap);
        waitForStart();
        start();

        elapsedTime.startTime();

        while (opModeIsActive())
        {
            telemetry.addData("Running, time elapsed", elapsedTime.time());
            telemetry.update();
        }
    }


    public class thread extends Thread
    {
        @Override
        public void run()
        {
            try
            {
                while (!interrupted())
                {
                    conceptTensorFlowObjectDetectionWebcamSam.detect(telemetry);
                }
            }
            catch (Exception e) {System.out.println(e.toString());}
        }
    }
}
