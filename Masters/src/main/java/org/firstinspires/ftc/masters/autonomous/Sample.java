package org.firstinspires.ftc.masters.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

@Autonomous(name="Sample")
public class Sample extends LinearOpMode {


    Pose startPose = new Pose(10,110,0);
    Pose bucketPose = new Pose (14,128,-45);
    Pose sample1 = new Pose(14,128,-14);
    Pose sample2 = new Pose(14,128,-5);
    Pose sample3 = new Pose(14, 128, 23);

    Path scorePreload;

    Follower follower;



    enum PathState {ToBucket, }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime elapsedTime = null;

        Init init = new Init(hardwareMap);
        Outtake outtake = new Outtake(init, telemetry);
        Intake intake = new Intake(init, telemetry);




        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        outtake.initAutoSample();

        waitForStart();

        outtake.scoreSample();
        elapsedTime = new ElapsedTime();
        PathState pathState =PathState.ToBucket;
        follower.followPath(scorePreload);


        while (opModeIsActive() && !isStopRequested()) {
            switch (pathState){
                case ToBucket:
                    if (!follower.isBusy() && outtake.isLiftReady() && !outtake.isScoringDone()){
                        outtake.releaseSample();
                    } else if (!follower.isBusy() && outtake.isScoringDone()){

                    }

                    break;
            }


            outtake.update();
            intake.update();
            follower.update();
        }

    }

    protected void buildPaths(){

    }
}
