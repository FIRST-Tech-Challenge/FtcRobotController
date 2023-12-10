package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoOut;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class BlueNear extends LinearOpMode {
    public static boolean parkCenter = false; // Park center of field
    public static boolean IS_RED = false;     // IS_RED side?
    public static boolean ALIGN_RIGHT = false; // Align 1 inch from tile right side
    public static double POS1_SPL1_X = 24;
    public static double POS1_SPL1_Y = 24;
    public static double POS1_DUMP_X = 24;
    public static double POS1_DUMP_Y = 33;

    public static double POS2_SPL1_X = 37;
    public static double POS2_SPL1_Y = 15;
    public static double POS3_SPL1_X = 26;
    public static double POS3_SPL1_Y = 26;
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(92);
    public static double PARK_STRAFE_MIDDLE_TO_CENTER = 28;
    public static double TAG_DIST = 6;
    public static double PARK_FORWARD = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        RobotVision rvis = new RobotVision(ALIGN_RIGHT);

        // general variable
        int elementPos;

        // Commands
        //Servo init code here
        robot.intake.toBasePos();
        robot.outtake.toIntakePos();
        dropIntakePreload dropIntakePreload = new dropIntakePreload(robot);
        autoOut           outCmd = new autoOut(robot);

        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        Log.v("AUTODEBUG", "0: start");
        elementPos = rvis.getTeamPropOrientation(IS_RED, ALIGN_RIGHT);
        //Log.v("AUTODEBUG", "1: elementPos = %0d");
        telemetry.addData("Element pos", elementPos);


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(23, 24), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .forward(16)
                            .build()
            ));
            //dump yellow pixel
            robot.runCommand(outCmd);
            Log.v("AUTODEBUG", "10: dump done");

            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeLeft(PARK_STRAFE_MIDDLE_TO_CENTER - TAG_DIST)
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeRight(PARK_STRAFE_MIDDLE_TO_CENTER + TAG_DIST)
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS2_SPL1_X, POS2_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(22, 35), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump yellow pixel
            robot.runCommand(outCmd);
            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeLeft(PARK_STRAFE_MIDDLE_TO_CENTER)
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeRight(PARK_STRAFE_MIDDLE_TO_CENTER)
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }

        } else {// right
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS3_SPL1_X, POS3_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(16, 35), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump yellow pixel
            robot.runCommand(outCmd);
            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeLeft(PARK_STRAFE_MIDDLE_TO_CENTER + TAG_DIST)
                                 .forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .strafeRight(PARK_STRAFE_MIDDLE_TO_CENTER - TAG_DIST)
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }
        }

    }
}
