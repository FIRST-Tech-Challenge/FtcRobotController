package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Autonomous(name = "Power Play Right")
public class PowerPlayRight extends LinearOpMode {

    enum State {
        SCORE_1,
        SCORE_2,
        NEW_CONE_FROM_SCORE_1,
        SCORE_CONE,
        NEW_CONE,
        PARK_GRAY,
        PARK_RED,
        PARK_GREEN

    }

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(37.5, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        drive.closeClaw();



        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory startToFirstDeposit = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d( new Vector2d(14,-60), Math.toRadians(90)), Math.toRadians(180))
                .build();

        Trajectory startToFirstDeposit2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(10.75,-35.5),Math.toRadians(135)))
                .build();

        Trajectory firstDepositToConeStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                // Ideally just a turn -45
                .splineToLinearHeading(new Pose2d( new Vector2d(20,-14), Math.toRadians(0)),Math.toRadians(0))
                .build();

        Trajectory scoreNewCone = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(32.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory fromScoreNewConeToConeStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
                .build();

        Trajectory parkGray = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(11.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(23.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        waitForStart();

        drive.closeClaw();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }

        currentState = State.SCORE_1;
        drive.followTrajectoryAsync(startToFirstDeposit);

        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                    telemetry.addData("First trajectory","");
                    telemetry.update();
                    if (!drive.isBusy()) {
                        telemetry.addData("Finished first trajectory","");
                        telemetry.update();
                        currentState = State.SCORE_2;
//                        drive.followTrajectoryAsync(startToFirstDeposit2);
                    }
                    break;
                case SCORE_2:
//                    telemetry.addData("Second trajectory","");
//                    telemetry.update();
//                    if (!drive.isBusy()) {
//                        currentState = State.NEW_CONE_FROM_SCORE_1;
//                        drive.turnAsync(Math.toRadians(-45));
//                        drive.followTrajectoryAsync(firstDepositToConeStack);
//                    }
//                    break;
//                case NEW_CONE_FROM_SCORE_1:
//                    telemetry.addData("Set up grab new cone","");
//                    telemetry.update();
//                    if (!drive.isBusy()) {
//                        currentState = State.SCORE_CONE;
//                        drive.followTrajectoryAsync(scoreNewCone);
//                    }
//                    break;
//                case SCORE_CONE:
//                    if (!drive.isBusy() && getRuntime() < 900000000) {
//                        currentState = State.NEW_CONE;
//                        drive.followTrajectoryAsync(fromScoreNewConeToConeStack);
//
//                    } else if(!drive.isBusy()) {
//
//                        if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
//                            currentState = State.PARK_GRAY;
//                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
//                            currentState = State.PARK_GREEN;
//                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
//                            currentState = State.PARK_RED;
//                        }
//                    }
//                    break;
//                case NEW_CONE:
//                    if (!drive.isBusy()) {
//                        currentState = State.SCORE_CONE;
//                        drive.followTrajectoryAsync(scoreNewCone);
//                    } else if(getRuntime() == 900000000) {
//
//                    }
//                    break;
//                case PARK_GRAY:
//                    drive.followTrajectoryAsync(parkGray);
//                    if (!drive.isBusy()) {
//
//                    }
//                    break;
//                case PARK_RED:
//                    drive.followTrajectoryAsync(parkRed);
//                    if (!drive.isBusy()) {
//
//                    }
//                    break;
//                case PARK_GREEN:
//                    break;
            }
        }

    }

}
