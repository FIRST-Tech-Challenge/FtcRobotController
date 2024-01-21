package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class RedFar extends LinearOpMode {
    public static boolean parkCenter = false; // Park center of field
    public static boolean IS_RED = true;     // IS_RED side?
    public static boolean ALIGN_RIGHT = false; // Align 1 inch from tile right side
    public static double drivePwr = 0.2;
    public static double hCoeff = 5;
    public static double park_y = -80;

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
        //Log.v("AUTODEBUG", "1: elementPos = " + elementPos);
        //telemetry.addData("Element pos", elementPos);
        telemetry.addData("Is parking center?: ", parkCenter);


        if (elementPos == 1) {//left

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(11+4, 10), Math.toRadians(-180))
                            .build()
            ));
            //Log.v("AUTODEBUG", "2: dump purple");
            //dump purple pixel
            robot.runCommand(dropIntakePreload);
            //Log.v("AUTODEBUG", "3: go to backdrop");

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .turn(Math.toRadians(45), 1,0.5)
                            .splineTo(new Vector2d(-2, -2), Math.toRadians(-90))
                            .splineTo(new Vector2d(-2, -68), Math.toRadians(-90))
                            .splineTo(new Vector2d(31, park_y), Math.toRadians(-90)) // go to backdrop
                            //.lineTo(new Vector2d(2, -74))
                            .build()
            ));
            //Log.v("AUTODEBUG", "4: dump yellow");
            robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            //Log.v("AUTODEBUG", "5: park");
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(50, park_y))
                                //.lineTo(new Vector2d(2, 4))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(-4, -81))
                                .build()
                ));
            }

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(21, 6), Math.toRadians(-180))
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(-2, -2), Math.toRadians(-90))
                            .splineTo(new Vector2d(-2, -68), Math.toRadians(-90))
                            .splineTo(new Vector2d(22, park_y), Math.toRadians(-90))
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
                                .lineTo(new Vector2d(50, -81))
                                //.lineTo(new Vector2d(2, 4))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(-4, -81 ))
                                .build()
                ));
            }

        } else {// right

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(24+4, 7), Math.toRadians(90))
                            .back(6)
                            .build()
            ));
            //Log.v("AUTODEBUG", "2: dump purple");
            //dump purple pixel
            robot.runCommand(dropIntakePreload);
            //Log.v("AUTODEBUG", "3: go to backdrop");

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .turn(Math.toRadians(45), 1,0.5)
                            .splineTo(new Vector2d(-2, -2), Math.toRadians(-90))
                            .splineTo(new Vector2d(-2, -68), Math.toRadians(-90))
                            .splineTo(new Vector2d(16, park_y), Math.toRadians(-90)) // go to backdrop
                            //.lineTo(new Vector2d(2, -74))
                            .build()
            ));
            //Log.v("AUTODEBUG", "4: dump yellow");
            robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            //Log.v("AUTODEBUG", "5: park");
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(50, -81))
                                //.lineTo(new Vector2d(2, 4))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(-4, -81 ))
                                .build()
                ));
            }


        }

    }
}
