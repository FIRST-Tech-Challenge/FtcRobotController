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
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class BluePark extends LinearOpMode {
    int bark = 1;
    public static double followRad1=5, followRad2 = 5, followRad3 =5, x1 = 10.5, y1 = 44, x2 = 95, y2 = -30, x3 = 48, y3= 0.0005,
            pow1 = 0.3, pow2 = 0.55, pow3 = 0.55, buff1 = 4, buff2 = 3, buff3 = 3,
            followRad4 = 5 ,x4 = 16.5, y4 = 47, pow4 = 0.55, buff4 = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(17, 63, Math.toRadians(90)));
        Path[] toSpike = new Path[3];
        Waypoint start = new StartWaypoint( new com.arcrobotics.ftclib.geometry.Pose2d(
                17, 64, new Rotation2d(toRadians(90))));
        toSpike[0] = new Path(start);
        toSpike[0].add(new GeneralWaypoint(x4,y4, toRadians(x2), pow4, .2,followRad1));
        toSpike[0].add(new EndWaypoint(x1, y1, toRadians(-x3), pow3, 0.2, 8, buff1, toRadians(30)));
        toSpike[1] = new Path(start);
        toSpike[1].add(new EndWaypoint(16.5, 47, toRadians(91), 0.7, 0, 5, 2, toRadians(10)));
        toSpike[2] = new Path(start);
        toSpike[2].add(new EndWaypoint(24.5, 49, toRadians(90), 0.6, 0.2, 5, 2, toRadians(10)));
        Path[] spikeToBackdrop = new Path[3];

        Path[] preToStack = new Path[3];
        Path stackToBack = new Path();

        robot.setRight(true);
        robot.observeSpike();
        while (!isStarted() || isStopRequested()) {
            bark = robot.getSpikePos();
            telemetry.addData("pixel", bark);
            packet.put("pix", bark);
            robot.update();
        }
        bark=0;
        while (!isStopRequested() && opModeIsActive()) {
//            robot.queuer.queue(false, true);
//            robot.upAuto();
//            robot.purpurAuto();
//            robot.queuer.addDelay(3.5);
//            robot.followPPPath(toSpike[bark]);
//            robot.queuer.addDelay(0.9);
//            robot.dropAuto(0);
//            if (bark == 0) {
//                spikeToBackdrop[0] = new Path();
//                spikeToBackdrop[0].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX()+1, currentPose.getY())));
//                spikeToBackdrop[0].add(
//                        new EndWaypoint(48, 28, toRadians(179), .6, .1, 3, 3, toRadians(10)));
//            }
//            if (bark == 1) {
//                spikeToBackdrop[1] = new Path();
//                spikeToBackdrop[1].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX()+1, currentPose.getY())));
//                spikeToBackdrop[1].add(
//                        new EndWaypoint(48, 36, toRadians(179), .55, 0.2, 5, 3, toRadians(5)));
//            }
//
//            if (bark ==2){
//                spikeToBackdrop[2] = new Path();
//                spikeToBackdrop[2].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX()+1, currentPose.getY())));
//                spikeToBackdrop[2].add(
//                        new EndWaypoint(48, 42, toRadians(179), .55, 0.2, 3, 3, toRadians(5)));
//            }
//            robot.followPPPath(spikeToBackdrop[bark]);
//            robot.queuer.addDelay(0.3);
//            robot.veryLowAuto();
//            robot.drop();
//            if (bark == 0) {
//                preToStack[0] = new Path();
//                preToStack[0].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//                preToStack[0].add(
//                        new EndWaypoint(48-2, 28, toRadians(179), pow2, 0.1, 5, 3, toRadians(10)));
//            }
//            if (bark == 1) {
//                preToStack[1] = new Path();
//                preToStack[1].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//                preToStack[1].add(
//                        new EndWaypoint(45, 36, toRadians(179), .55, 0.2, 5, 3, toRadians(5)));
//            }
//            if (bark ==2){
//                preToStack[2] = new Path();
//                preToStack[2].add(
//                        new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//                preToStack[2].add(
//                        new EndWaypoint(45, 42, toRadians(179), .55, 0.2, 5, 3, toRadians(5)));
//            }
//
//            robot.followPPPath(preToStack[bark]);
//      for (int i = 0; i < 3; i++) {
            // move away
//      robot.queuer.addDelay(0.5);
//      robot.resetAuto();
//        robot.queuer.addDelay(2.5);
//        robot.intakeAuto(4);
//        // move to backdrop
//        robot.lowAuto();
//        robot.drop();
//      }
//      //move to park
//            robot.queuer.addDelay(0.3);
//            robot.resetAuto();
//      robot.intakeAuto(5);
//      robot.queuer.waitForFinish();
//      robot.followPPPath(stackToBack);
//      robot.lowAuto();
//      robot.drop();
//      robot.resetAuto();
            preToStack[2] = new Path();
            preToStack[2].add(
                    new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
            preToStack[2].add(
                    new EndWaypoint(50, 60, toRadians(-179), .55, 0.2, 5, 3, toRadians(5)));

            robot.followPPPath(preToStack[2]);
            robot.update();
        }
    }
}
