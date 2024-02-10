package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class BlueLeft20 extends LinearOpMode {
    int bark = 1;
    public static double x1 = -35, y1 = 36, h1 = 180, v1 = 0.5, w1 = 0.1, fR1 = 6,bf1 = 2,
            x2 = -54.5, y2 = 35, h2 = -180, v2 = 0.35, w2 = 0.1, fR2 = 4, bf2 = 1,
            x3 = -54.5, y3 = 36, h3 = -190, v3 = 0.35, w3 = 0.1, fR3 = 2, bf3 = 1,
            xx1 = 9, yy1 = -58, hh1 = -160, vv1 = 1.0, ww1 = 0.2, ffR1 = 8,
            xx2 = -29, yy2 = -58, hh2 = -185, vv2 = 1.0, ww2 = 0.2, ffR2 = 6,
            xx3 = -51, yy3 = -42, hh3 = -210, vv3 = 0.9, ww3 = 0.2, ffR3 = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(-38, 61, Math.toRadians(90)));
        Path[] toSpike = new Path[3];
        Waypoint start =
                new StartWaypoint(
                        new com.arcrobotics.ftclib.geometry.Pose2d(-38, 61, new Rotation2d(toRadians(90))));
        toSpike[0] = new Path(start);
        toSpike[0].add(new GeneralWaypoint(-40, 50, toRadians(70), 0.4, 0.3, 5));
        toSpike[0].add(new EndWaypoint(-41, 39, toRadians(70), 0.4, 0.2,5, 1, toRadians(10)));
        toSpike[1] = new Path(start);
        toSpike[1].add(new EndWaypoint(-38, 41, toRadians(91), 0.5, 0, 5, 2, toRadians(10)));;
        toSpike[2] = new Path(start);
        toSpike[2].add(new GeneralWaypoint(-38, 46, toRadians(90), 0.4, 0.3, 5));
        toSpike[2].add(new EndWaypoint(x1, y1, toRadians(h1), v1, w1, fR1, bf1, toRadians(10)));
        Path[] spikeToBackdrop = new Path[3];
        Path[] preToStack = new Path[3];

        robot.dropServo(1);
        robot.setRight(true);
        robot.observeSpike();

        while (!isStarted() || isStopRequested()) {
            bark = robot.getSpikePos();
            telemetry.addData("pixel", bark);
            packet.put("pix", bark);
            robot.update();
        }
        while (!isStopRequested() && opModeIsActive()) {
            robot.queuer.queue(false, true);
            robot.upAuto();
            robot.purpurAuto();
            robot.queuer.addDelay(2.5);
            robot.followPPPath(toSpike[2-bark]);
            robot.queuer.addDelay(0.9);
            robot.dropAuto(0);

            if (bark == 0) {
                preToStack[0] = new Path();
                preToStack[0].add(
                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
                preToStack[0].add(new GeneralWaypoint(-39, 59, toRadians(200), 1.0, 0.3, 6));
                preToStack[0].add(new GeneralWaypoint(16, 59, toRadians(185), 1.0, 0.3, 6));
                preToStack[0].add(new GeneralWaypoint(40, 47, toRadians(180), 0.9, 0.3, 6));
                preToStack[0].add(
                        new EndWaypoint(45.5, 46, toRadians(180), 0.5, 0.2, 5, 2, toRadians(10)));
            }
            if (bark == 1) {
                Path preBack = new Path();
                preBack.add(new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(toRadians(90)))));
                preBack.add(new EndWaypoint(currentPose.getX(), currentPose.getY() + 5,toRadians(90),0.4, 0.1,2,1,toRadians(10)));
                robot.queuer.addDelay(0.8);
                robot.followPPPath(preBack);
                preToStack[1] = new Path();
                preToStack[1].add(
                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
                preToStack[1].add(new GeneralWaypoint(-39, 59, toRadians(200), 1.0, 0.3, 6));
                preToStack[1].add(new GeneralWaypoint(16, 59, toRadians(185), 1.0, 0.3, 6));
                preToStack[1].add(new GeneralWaypoint(40, 40, toRadians(180), 0.9, 0.3, 6));
                preToStack[1].add(
                        new EndWaypoint(45.5, 39.5, toRadians(178), .5, 0.3, 5, 2, toRadians(5)));
            }
            if (bark ==2){
                Path preBack = new Path();
                preBack.add(new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(toRadians(90)))));
                preBack.add(new EndWaypoint(currentPose.getX(), currentPose.getY() + 8,toRadians(90),0.4, 0.1,2,1,toRadians(10)));
                robot.queuer.addDelay(0.8);
                robot.followPPPath(preBack);
                preToStack[2] = new Path();
                preToStack[2].add(
                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
                preToStack[2].add(new GeneralWaypoint(-39, 59, toRadians(200), 1.0, 0.3, 6));
                preToStack[2].add(new GeneralWaypoint(16, 59, toRadians(185), 1.0, 0.3, 6));
                preToStack[2].add(new GeneralWaypoint(40, 33, toRadians(180), 0.9, 0.3, 6));
                preToStack[2].add(
                        new EndWaypoint(45.5, 29, toRadians(179), .4, 0.3, 5,2, toRadians(5)));
            }
            robot.queuer.queue(false, true);
            robot.resetAuto();
            robot.queuer.waitForFinish();
            robot.queuer.addDelay(1.0);
            robot.followPPPath(preToStack[bark]);
            robot.grabSupAuto();
            robot.queuer.waitForFinish();
            robot.queuer.queue(false, true);
            robot.lowAuto();
            robot.queuer.waitForFinish();
            robot.queuer.addDelay(0.5);
            robot.drop();

            Path back = new Path();
            back.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
            back.add(new EndWaypoint(currentPose.getX()-6, currentPose.getY(),toRadians(180),0.4, 0.1,5,3,toRadians(10)));
            robot.queuer.addDelay(0.8);
            robot.followPPPath(back);

            preToStack[2] = new Path();
            preToStack[2].add(
                    new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
            preToStack[2].add(
                    new EndWaypoint(44, 13, toRadians(179), .5, 0.2, 5, 3, toRadians(5)));

            robot.followPPPath(preToStack[2]);
//            Path pak = new Path();
//            pak.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//            pak.add(new EndWaypoint(46, 13, toRadians(179), .5, 0.2, 5, 3, toRadians(5)));
//            robot.followPPPath(pak);


            robot.resetAuto();
            robot.update();
        }
    }
}
