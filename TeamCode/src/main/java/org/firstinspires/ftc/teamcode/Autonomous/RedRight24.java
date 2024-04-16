package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedRight2+4")
@Config
public class RedRight24 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RRMAX aut = new RRMAX(this, false);
        aut.waitForStart();
        while (!isStopRequested() && opModeIsActive()&& aut.isAutDone()) {
            aut.purp();
            aut.pre();
            aut.cycleIntake(5);
            aut.cycleDrop();
            aut.cycleIntake2(3);
            aut.cycleDrop();
            aut.cycleIntake3(5);
            aut.cycleDrop2();
            aut.cycleIntake4(3);
            aut.cycleDrop2();
            aut.park();
            aut.update();
        }
        stop();
    }
}
