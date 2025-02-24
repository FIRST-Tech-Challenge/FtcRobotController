package org.firstinspires.ftc.masters.test;

import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

public class RobotLocation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Constants.setConstants(FConstants.class, LConstants.class);
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(new Pose(10,66,0));

        Init init = new Init(hardwareMap);
        Intake intake = new Intake(init, telemetry);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                intake.extendSlideMax();
                intake.dropIntake();

            }
            if (gamepad1.b){
                intake.retractSlide();
            }

            if (gamepad1.x){
                intake.toNeutral();
            }

            telemetry.addData("x", poseUpdater.getPose().getX());
            telemetry.addData("y", poseUpdater.getPose().getY());
            telemetry.addData("heading", poseUpdater.getPose().getHeading());
            telemetry.update();

        }
    }
}
