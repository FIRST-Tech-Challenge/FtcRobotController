package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;

@Config
@Autonomous
public class RedNear extends LinearOpMode {
    public static boolean parkCenter = false;
    public static boolean IS_RED = true;
    public static boolean ALIGN_RIGHT = true;
    public static double POS1_SPL1_X = 25;
    public static double POS1_SPL1_Y = -5;
    public static double POS1_DUMP_X = 32;
    public static double DUMP_Y = -25;

    public static double POS2_SPL1_X = 37;
    public static double POS2_SPL1_Y = -15;
    public static double POS3_SPL1_X = 26;
    public static double POS3_SPL1_Y = -23;
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(-90);
    public static double PARK_CENTER_X = 50;
    public static double PARK_CORNER_X = 0;
    public static double PARK_STRAFE_MIDDLE_TO_CENTER = 28;
    public static double TAG_DIST = 6;
    public static double PARK_FORWARD = 10.0;
    public static double drivePwr = 0.2;
    public static double hCoeff = 5;

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
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
        //Servo init code here
        robot.intake.toBasePos();
        robot.outtake.toIntakePos();
        dropIntakePreload dropIntakePreload = new dropIntakePreload(robot);
        autoOutPrep outPrep = new autoOutPrep(robot);
        autoOutDump outDump = new autoOutDump(robot);

        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        telemetry.addData("Is parking center?: ", parkCenter);
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        //Log.v("AUTODEBUG", "0: start");
        elementPos = rvis.getTeamPropOrientation(IS_RED, ALIGN_RIGHT);
        //Log.v("AUTODEBUG", "1: elementPos = %0d");
        //telemetry.addData("Element pos", elementPos);
        telemetry.addData("Is parking center?: ", parkCenter);


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS1_SPL1_X+5, POS1_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(POS1_SPL1_X+5, POS1_SPL1_Y+2))
                            .build()
            ));

            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(POS1_DUMP_X, DUMP_Y), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(POS1_DUMP_X, DUMP_Y))
                            .build()
            ));
            robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            //Log.v("AUTODEBUG", "10: dump done");

            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
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
                            //.splineTo(new Vector2d(24, -35), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(23, DUMP_Y))
                            .build()
            ));
            robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
                                .build()
                ));
            }

        } else {// right
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS3_SPL1_X+5, POS3_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(17, -35), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(18, DUMP_Y))
                            .build()
            ));
            robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-10))
                                //.forward(PARK_FORWARD)
                                .build()
                ));
            }
        }

    }
}
