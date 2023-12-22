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
public class outCmdTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        // Commands
        //Servo init code here
        robot.intake.toBasePos();
        //robot.outtake.toIntakePos();
        autoOut           outCmd = new autoOut(robot);

        // Start
        waitForStart();
        if (isStopRequested()) return;
        //Log.v("AUTODEBUG", "0: start");

        robot.runCommand(outCmd);
    }
}
