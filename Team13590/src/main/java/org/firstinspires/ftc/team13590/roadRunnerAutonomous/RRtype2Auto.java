package org.firstinspires.ftc.team13590.roadRunnerAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13590.RRactions;
import org.firstinspires.ftc.team13590.RobotHardware;
import org.firstinspires.ftc.team13590.teleop.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name = "RR Ascent Zone Right", group = "RoadRunner")
public class RRtype2Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(23, -61.25, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action leg1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(50))
                .splineToConstantHeading(new Vector2d(24, 24), Math.toRadians(180))
                .waitSeconds(1)
                .build();

        robot.init(true);

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        leg1
                        // leg 2 actions

                )
        );
    }
}
