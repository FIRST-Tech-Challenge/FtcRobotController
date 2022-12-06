package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Autonomous(name = "Power Play Blue Left")
public class PowerPlayBlueLeft extends LinearOpMode{

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }


        //When parking
        switch (sleeveColor) {
            case RED:
                //Parking 2
            case GRAY:
                //Parking 1
            case GREEN:
                //Parking 3
            case INDETERMINATE:

        }
    }

}
