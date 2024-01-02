package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoOut;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class AlignTest extends LinearOpMode {
    public static double drivePwr = 0.01;
    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        // general variable
        int elementPos;

        // Commands
        //Servo init code here
        robot.intake.toBasePos();
        robot.outtake.toIntakePos();
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, new Pose2d(drivePwr,0,0),15, telemetry);

        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;

        robot.runCommand(alignCmd);
    }
}
