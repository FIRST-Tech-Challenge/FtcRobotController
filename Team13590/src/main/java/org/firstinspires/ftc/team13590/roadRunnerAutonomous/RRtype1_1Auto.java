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
@Autonomous(name = "RR Specimen Test", group = "RoadRunner")
public class RRtype1_1Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -64.5, Math.toRadians(90)); // subtracted 3.25 in y
        Pose2d initialRungPose = new Pose2d(9.5, -30, Math.toRadians(90));

        Pose2d rungPose = new Pose2d(9.5, -43.25, Math.toRadians(90)); // subtracted 3.25 in y
        Pose2d rungPose2 = new Pose2d(11, -42.5, Math.toRadians(90));
        Pose2d rungPose3 = new Pose2d(7, -42.25, Math.toRadians(90));
        Pose2d rungPose4 = new Pose2d(5, -42.25, Math.toRadians(90));

        Pose2d sample1Pose = new Pose2d(38, -38, Math.toRadians(50));
        Pose2d drop1Pose = new Pose2d(46.5,-43, Math.toRadians(90));

        Pose2d sample2Pose = new Pose2d(46.5, -38, Math.toRadians(50));
        Pose2d drop2Pose = new Pose2d(57, -43, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(57, -38, Math.toRadians(50));
        //Pose2d drop3Pose = new Pose2d(48, -55.1, Math.toRadians(90));

        Pose2d pickupPose = new Pose2d(38, -55.1 + 0.3, Math.toRadians(90));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action score1 = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(initialRungPose.position.y)
                .build();

        Action backup = drive.actionBuilder(initialRungPose)
                .setTangent(Math.atan2(rungPose.position.y - initialRungPose.position.y, rungPose.position.x - initialPose.position.x))
                .lineToY(rungPose.position.y)
                .build();

        Action grabSample1 = drive.actionBuilder(rungPose)
                .setTangent(Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x)) // just the atan (y2-y1 / x2-x1)
                .lineToX(sample1Pose.position.x, null, new ProfileAccelConstraint(-50, 70))
                .build();

        Action dropSample1 = drive.actionBuilder(sample1Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .lineToX(drop1Pose.position.x)
                .build();

        Action grabSample2 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .lineToY(sample2Pose.position.y)
                .build();

        Action dropSample2 = drive.actionBuilder(sample2Pose)
                .setTangent(Math.atan2(drop2Pose.position.y - sample2Pose.position.y, drop2Pose.position.x - sample2Pose.position.x))
                .lineToY(drop2Pose.position.y)
                .build();

        Action grabSample3 = drive.actionBuilder(drop2Pose)
                .setTangent(Math.atan2(sample3Pose.position.y - drop2Pose.position.y, sample3Pose.position.x - drop2Pose.position.x))
                .lineToYLinearHeading(sample3Pose.position.y, sample3Pose.heading)
                .build();

        Action dropSample3 = drive.actionBuilder(sample3Pose)
                .setTangent(Math.atan2(pickupPose.position.y - sample3Pose.position.y, pickupPose.position.x - sample3Pose.position.x))
                .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)
                .afterTime(0.2, pinch.openClaw())
                .build();

        Action score2 = drive.actionBuilder(pickupPose)
                .setTangent(Math.atan2((rungPose2.position.y) - pickupPose.position.y, rungPose2.position.x - pickupPose.position.x))
                .lineToXLinearHeading(rungPose2.position.x, rungPose3.heading, null, new ProfileAccelConstraint(-30, 70))
                .build();

        Action pickup2 = drive.actionBuilder(rungPose2)
                .setTangent(Math.atan2(pickupPose.position.y - rungPose2.position.y, pickupPose.position.x - rungPose2.position.x))
                .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)
                .build();

        Action score3 = drive.actionBuilder(pickupPose)
                .setTangent(Math.atan2(rungPose3.position.y - pickupPose.position.y, rungPose3.position.x - pickupPose.position.x))
                .lineToXLinearHeading(rungPose3.position.x, rungPose3.heading, null, new ProfileAccelConstraint(-30, 70))
                .build();

        Action pickUp3 = drive.actionBuilder(rungPose3)
                .setTangent(Math.atan2(pickupPose.position.y - rungPose3.position.y, pickupPose.position.x - rungPose3.position.x))
                .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)
                .build();

        Action score4 = drive.actionBuilder(pickupPose)
                .setTangent(Math.atan2(rungPose4.position.y - pickupPose.position.y, rungPose4.position.x - pickupPose.position.x))
                .lineToXLinearHeading(rungPose4.position.x, rungPose4.heading, null, new ProfileAccelConstraint(-30, 70))
                .build();

        Action park = drive.actionBuilder(rungPose4)
                .setTangent(Math.atan2(pickupPose.position.y - rungPose4.position.y, pickupPose.position.x - rungPose4.position.x))
                .lineToX(pickupPose.position.x, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 80))
                .build();


        robot.init(true);

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(  // go to score first specimen
                                score1,
                                elbow.elbowToDeg(80),
                                extension.extenderToInch(0.3),
                                axial.rotateAxial(robot.CLAW_MID),
                                yaw.rotateClaw(robot.YAW_RIGHT)
                        ),
                        elbow.elbowToDeg(121),
                        pinch.openClaw(),
                        backup,

                        // go to next sample and pick up
                        new ParallelAction(
                                grabSample1,
                                elbow.elbowToDeg(20),
                                yaw.rotateClaw(0.45), // 0.37 + 0.08
                                extension.extenderToInch(8.25),
                                axial.rotateAxial(0.25)
                        ),
                        elbow.elbowToDeg(16),
                        pinch.closeClaw(),
                        sleepAction(400),

                        new ParallelAction( // drop off sample
                                dropSample1,
                                elbow.elbowToDeg(30),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),
                        sleepAction(200),

                        new ParallelAction(
                                grabSample2,
                                elbow.elbowToDeg(20),
                                yaw.rotateClaw(0.45), // 0.37 + 0.08
                                extension.extenderToInch(8.25),
                                axial.rotateAxial(0.25)
                        ),
                        elbow.elbowToDeg(16),
                        pinch.closeClaw(),
                        sleepAction(400),

                        new ParallelAction( // drop off sample
                                dropSample2,
                                elbow.elbowToDeg(30),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),
                        sleepAction(200),


                        grabSample3,
                        new ParallelAction( // go to 3rd sample, rotate bot & claw
                                elbow.elbowToDeg(20),
                                yaw.rotateClaw(0.45), // 0.37 + 0.08
                                extension.extenderToInch(8.25),
                                axial.rotateAxial(0.25)
                        ),
                        elbow.elbowToDeg(16),
                        pinch.closeClaw(),
                        sleepAction(400),

                        new ParallelAction( // retract and drop off 3rd sample
                                dropSample3,
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(180+37),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),


                        // 2nd specimen
                        new ParallelAction( // pick up specimen FROM WALL
                                elbow.elbowToDeg(180+33),
                                extension.extenderToInch(2.3),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),

                        sleepAction(200),
                        pinch.closeClaw(),

                        new ParallelAction( // drop off 2nd specimen
                                score2,
                                elbow.elbowToDeg(112),
                                axial.rotateAxial(robot.CLAW_UP),
                                new SequentialAction(
                                        sleepAction(300),
                                        extension.extenderToInch(8.5)
                                ),
                                new SequentialAction(
                                        sleepAction(300),
                                        yaw.rotateClaw(robot.YAW_RIGHT)
                                )
                        ),
                        yaw.rotateClaw(robot.YAW_RIGHT),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),
                        pinch.openClaw(),

                        // 3rd specimen
                        new ParallelAction( // retract & come back for pickup FROM WALL
                                extension.extenderToInch(2.3),
                                elbow.elbowToDeg(180+29),
                                axial.rotateAxial(robot.CLAW_MID),
                                pickup2
                        ),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // score 3rd specimen
                                score3,
                                elbow.elbowToDeg(112),
                                axial.rotateAxial(robot.CLAW_UP),
                                new SequentialAction(
                                        sleepAction(300),
                                        extension.extenderToInch(8.5)
                                ),

                                extension.extenderToInch(8.5),
                                new SequentialAction(
                                        sleepAction(300),
                                        yaw.rotateClaw(robot.YAW_MID)
                                )
                        ),
                        yaw.rotateClaw(robot.YAW_MID),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),
                        pinch.openClaw(),

                        // 4th specimen
                        new ParallelAction( // retract & come back for pickup FROM WALL
                                extension.extenderToInch(2.3),
                                elbow.elbowToDeg(180+29),
                                axial.rotateAxial(robot.CLAW_MID),
                                pickUp3
                        ),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // score 3rd specimen
                                score4,
                                elbow.elbowToDeg(112),
                                axial.rotateAxial(robot.CLAW_UP),

                                new SequentialAction(
                                        sleepAction(300),
                                        extension.extenderToInch(8.5)
                                ),
                                new SequentialAction(
                                        sleepAction(300),
                                        yaw.rotateClaw(robot.YAW_RIGHT)
                                )
                        ),
                        yaw.rotateClaw(robot.YAW_RIGHT),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),
                        pinch.openClaw(),

                        // come back & retract for parking
                        new ParallelAction(
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(37),
                                park
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
