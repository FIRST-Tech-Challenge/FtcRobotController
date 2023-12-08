package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.commands.autoOut;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;

@Config
@Autonomous
public class RedClose extends LinearOpMode {
    public static double MID_POLE_X = 31;
    public static double MID_POLE_HT = 18;

    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        RobotVision rvis = new RobotVision();

        // general variable
        int elementPos = 3;

        // Commands
        //Servo init code here
        robot.intake.toBasePos();
        dropIntakePreload dropIntakePreload = new dropIntakePreload(robot);
        autoOut           outCmd = new autoOut(robot);

        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        Log.v("AUTODEBUG", "0: start");
        elementPos = rvis.getTeamPropOrientation(true);
        //Log.v("AUTODEBUG", "1: elementPos = %0d");
        telemetry.addData("Element pos", elementPos);
        elementPos = 1;
        // drop intake preload

        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(24, -3), Math.toRadians(-90))
                            .build()
            ));
        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(30, -15), Math.toRadians(-90))
                            .build()
            ));
        } else {// right
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(24, -27), Math.toRadians(-90))
                            .build()
            ));
        }

        robot.runCommand(dropIntakePreload);
        // Move to backdrop
        robot.runCommand(outCmd);



        // Move forward two tile




    }
}
