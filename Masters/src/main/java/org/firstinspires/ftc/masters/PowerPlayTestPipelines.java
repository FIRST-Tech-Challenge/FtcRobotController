package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;

@Config
@Autonomous(name = "Power Play CV Pipe")
public class PowerPlayTestPipelines extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        //PowerPlayComputerVisionPipelines.BlueStackPipeline.StackPosition pos = null;
        PowerPlayComputerVisionPipelines.SleevePipeline.PipePosition pos = null;
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap,telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(36, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        CV.setPipeDetectionFront();
        waitForStart();

        boolean centered = false;

        while (opModeIsActive()) {
            pos = CV.sleevePipeline.position;
            centered = drive.alignPole(pos);
            telemetry.addData("pos",pos);
            telemetry.update();
        }
//        CV.stopSleeveCamera();

    }
}
