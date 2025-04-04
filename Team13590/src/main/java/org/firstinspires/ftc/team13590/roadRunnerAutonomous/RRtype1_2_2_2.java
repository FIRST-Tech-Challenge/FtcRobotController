package org.firstinspires.ftc.team13590.roadRunnerAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13590.RRactions;
import org.firstinspires.ftc.team13590.RobotHardware;
import org.firstinspires.ftc.team13590.teleop.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name = "RR Bucket : Sample + Park", group = "RoadRunner MAIN")
public class RRtype1_2_2_2 extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-9.5 -23, -64.5, Math.toRadians(90));
        Pose2d rungPose = new Pose2d(-9.5, -43.25, Math.toRadians(90));

        Pose2d sample1Pose = new Pose2d(-47, -33.5, Math.toRadians(90));
        Pose2d drop1Pose = new Pose2d(-56,-60, Math.toRadians(50));

        Pose2d sample2Pose = new Pose2d(-56.5, -34, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(-57, -38, Math.toRadians(130));

        Pose2d pickUpPose = new Pose2d(32, -60, Math.toRadians(0));

        Pose2d parkSpot = new Pose2d(-24, 0, Math.toRadians(90));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action score1 = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(drop1Pose.position.y - initialPose.position.y, drop1Pose.position.x - initialPose.position.x))
                .lineToYLinearHeading(drop1Pose.position.y, drop1Pose.heading)
                .waitSeconds(0.25)
                .build();

        Action grabSample1 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample1Pose.position.y - drop1Pose.position.y, sample1Pose.position.x - drop1Pose.position.x)) // just the atan (y2-y1 / x2-x1)
                .lineToXLinearHeading(sample1Pose.position.x, sample1Pose.heading)
                .build();

        Action dropSample1 = drive.actionBuilder(sample1Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .lineToXLinearHeading(drop1Pose.position.x, drop1Pose.heading)
                .build();

        Action grabSample2 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .lineToYLinearHeading(sample2Pose.position.y, sample2Pose.heading)
                .build();

        Action dropSample2 = drive.actionBuilder(sample2Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample2Pose.position.y, drop1Pose.position.x - sample2Pose.position.x))
                .lineToYLinearHeading(drop1Pose.position.y, drop1Pose.heading)
                .build();

        Action grabSample3 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample3Pose.position.y - drop1Pose.position.y, sample3Pose.position.x - drop1Pose.position.x))
                .lineToYLinearHeading(sample3Pose.position.y, sample3Pose.heading)
                .build();

        Action dropSample3 = drive.actionBuilder(sample3Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample3Pose.position.y, drop1Pose.position.x - sample3Pose.position.x))
                .lineToXLinearHeading(drop1Pose.position.x, drop1Pose.heading)
                .build();


        Action toPark = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(parkSpot.position.y - drop1Pose.position.y, parkSpot.position.x - drop1Pose.position.x))
                .splineToLinearHeading(parkSpot, Math.toRadians(40), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 30))
                .build();

        robot.init(true);

        double dropOffAngle = 12;

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( // score off preload sample
                                score1,
                                elbow.elbowToDeg(37+90+dropOffAngle),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),

                        grabSample1, // go to next sample and pick up
                        new ParallelAction(
                                extension.extenderToInch(0.9),
                                elbow.elbowToDeg(0),
                                axial.rotateAxial(robot.CLAW_DOWN)
                        ),

                        sleepAction(200),
                        pinch.closeClaw(),
                        sleepAction(400),

                        new ParallelAction( // score off sample
                                dropSample1,
                                elbow.elbowToDeg(37+90+dropOffAngle),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID + 0.2)
                        ),
                        pinch.openClaw(),
                        sleepAction(400),

                        new ParallelAction( // go to next sample
                                grabSample2,
                                extension.extenderToInch(0.9),
                                elbow.elbowToDeg(0)
                        ),
                        axial.rotateAxial(robot.CLAW_DOWN),
                        sleepAction(200),
                        pinch.closeClaw(),
                        sleepAction(400),

                        new ParallelAction( // drop off 2nd sample
                                dropSample2,
                                elbow.elbowToDeg(37+90+dropOffAngle),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID + 0.2)
                        ),
                        pinch.openClaw(),
                        sleepAction(400),

                        new ParallelAction(
                                grabSample3,
                                extension.extenderToInch(3),
                                elbow.elbowToDeg(37)
                        ),

                        new ParallelAction( // go to 3rd sample, rotate bot & claw
                                elbow.elbowToDeg(20),
                                yaw.rotateClaw(robot.YAW_MID - 0.08),
                                extension.extenderToInch(8.25),
                                axial.rotateAxial(0.25)
                        ),
                        elbow.elbowToDeg(18),
                        pinch.closeClaw(),
                        sleepAction(200),

                        new ParallelAction(
                                dropSample3,
                                elbow.elbowToDeg(37+90+dropOffAngle),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID + 0.2)
                        ),
                        pinch.openClaw(),
                        sleepAction(400),

                        new ParallelAction(
                                toPark,
                                elbow.elbowToDeg(0),
                                extension.extenderToInch(0)
                        )

                        // leg 2 actions

                )
        );
    }
    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }
}
