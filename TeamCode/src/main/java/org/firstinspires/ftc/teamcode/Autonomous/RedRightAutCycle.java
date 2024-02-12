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
public class RedRightAutCycle extends LinearOpMode {
  int bark = 1;
  public static double x1 = -39, y1 = -36, h1 = -180, v1 = 0.5, w1 = 0.1, fR1 = 6,bf1 = 2,
      x2 = -54.5, y2 = -35, h2 = -180, v2 = 0.35, w2 = 0.1, fR2 = 4, bf2 = 1,
      x3 = -54.5, y3 = -36, h3 = -190, v3 = 0.35, w3 = 0.1, fR3 = 2, bf3 = 1,
      xx1 = 9, yy1 = -58, hh1 = -160, vv1 = 1.0, ww1 = 0.2, ffR1 = 8,
      xx2 = -29, yy2 = -58, hh2 = -185, vv2 = 1.0, ww2 = 0.2, ffR2 = 6,
      xx3 = -51, yy3 = -42, hh3 = -210, vv3 = 0.9, ww3 = 0.2, ffR3 = 4;

  @Override
  public void runOpMode() throws InterruptedException {
    BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(17, -61, Math.toRadians(-90)));
    Path[] toSpike = new Path[3];
    Waypoint start = new StartWaypoint( new com.arcrobotics.ftclib.geometry.Pose2d(
            17, -61, new Rotation2d(toRadians(-90))));
    toSpike[0] = new Path(start);
    toSpike[0].add(new GeneralWaypoint(16.5,-44, toRadians(-95), 0.55, .2,5));
    toSpike[0].add(new EndWaypoint(8.5, -44, toRadians(48), 0.55, 0.4, 8, 4, toRadians(30)));
    toSpike[1] = new Path(start);
    toSpike[1].add(new EndWaypoint(16.5, -44, toRadians(-91), 0.7, 0, 5, 2, toRadians(10)));
    toSpike[2] = new Path(start);
    toSpike[2].add(new EndWaypoint(24.5, -43, toRadians(-90), 0.6, 0.0, 8, 1, toRadians(10)));
    Path[] spikeToBackdrop = new Path[3];
    robot.dropServo(1);
    robot.setRight(false);
    robot.setBlue(false);
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
      robot.queuer.addDelay(3.5);
      robot.followPPPath(toSpike[bark]);
      robot.queuer.addDelay(0.9);
      robot.dropAuto(0);
      if (bark == 0) {
        Path spikeToBackd = new Path();
        spikeToBackd.add(
                new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(currentPose.getHeading()))));
        spikeToBackd.add(new EndWaypoint(currentPose.getX()+3, currentPose.getY(), currentPose.getHeading(), .5,0,3,2,toRadians(10)));
        robot.followPPPath(spikeToBackd);
        spikeToBackdrop[0] = new Path();
        spikeToBackdrop[0].add(
                new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//        spikeToBackdrop[0].add(new GeneralWaypoint(currentPose.getX()+3, currentPose.getY(), currentPose.getHeading(), .5,0,3));
        spikeToBackdrop[0].add(
                new EndWaypoint(48, -28.5, toRadians(-179), .45, .1, 6, 1, toRadians(10)));
      }
      if (bark == 1) {
        spikeToBackdrop[1] = new Path();
        spikeToBackdrop[1].add(
                new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(currentPose.getHeading()))));
//        spikeToBackdrop[1].add(new GeneralWaypoint(currentPose.getX(), currentPose.getY()-5, currentPose.getHeading(), .5,0,3));
        spikeToBackdrop[1].add(
                new EndWaypoint(49, -34, toRadians(-180), .45, 0.2, 14, 1, toRadians(10)));
      }

      if (bark ==2){
        Path spikeToBackd = new Path();
        spikeToBackd.add(
                new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(currentPose.getHeading()))));
        spikeToBackd.add(new EndWaypoint(currentPose.getX(), currentPose.getY()-5, currentPose.getHeading(), .5,0,3,2,toRadians(10)));
        robot.followPPPath(spikeToBackd);
        spikeToBackdrop[2] = new Path();
        spikeToBackdrop[2].add(
                new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        spikeToBackdrop[2].add(
                new EndWaypoint(49, -41.5, toRadians(-179), .4, 0.3, 14, 1, toRadians(10)));
      }
      robot.followPPPath(spikeToBackdrop[bark]);
      robot.queuer.addDelay(0.3);
      robot.veryLowAuto();
      robot.drop();
      robot.queuer.waitForFinish();
//      Path preToStack = new Path();
//      //            if(bark==1){
//      preToStack.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
//      preToStack.add(new EndWaypoint(x2, y2, toRadians(h2), v2, w2, fR2, bf2, toRadians(10)));
//      //            }
//      robot.queuer.addDelay(0.6);
//      robot.followPPPath(preToStack);
//      robot.queuer.addDelay(.6);
//      robot.resetAuto();
//      robot.intakeAuto(5);
//      robot.queuer.waitForFinish();
//      Path stackToBack = new Path();
//      //            if(bark==1){
//      double y = 0, yy = 0;
//      if (bark == 0) {
//        y = -29;
//        yy = -31;
//      }
//      if (bark == 1) {
//        y = -36;
//        yy = -39;
//      }
//      if (bark == 2) {
//        y = -42;
//        yy = -43;
//      }
//
//      stackToBack.add(new StartWaypoint(new Translation2d(x2 + 2, y2)));
//      stackToBack.add(new GeneralWaypoint(-33, -56.5, toRadians(-220), 1.0, 0.4, 8));
//      stackToBack.add(new GeneralWaypoint(16, -56.5, toRadians(-180), 1.0, 0.4, 7));
//      stackToBack.add(new GeneralWaypoint(39, yy, toRadians(-160), 1.0, 0.4, 8));
//      stackToBack.add(new EndWaypoint(44.5, y, toRadians(-180), .35, .12, 5, 3, toRadians(10)));
//      //            }
//      robot.followPPPath(stackToBack);
//      robot.grabAuto();
//      robot.lowAuto();
//      robot.drop();
      for (int i = 0; i < 2; i++) {
        Path backToStack = new Path();
        backToStack.add(
            new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        backToStack.add(new GeneralWaypoint(xx1, yy1, toRadians(hh1), vv1, ww1, ffR1));
        backToStack.add(new GeneralWaypoint(xx2, yy2, toRadians(hh2), vv2, ww2, ffR2));
        backToStack.add(new GeneralWaypoint(xx3, yy3, toRadians(hh3), vv3, ww3, ffR3));
        backToStack.add(new EndWaypoint(x3, y3, toRadians(h3), v3, w3, fR3, bf3, toRadians(10)));
        //            Path park = new Path();
        ////            if(bark==1){
        //                park.add(new StartWaypoint(new Translation2d(currentPose.getX(),
        // currentPose.getY())));
        //                park.add(new EndWaypoint(35,-48,toRadians(-180),0.2,0,5,2,toRadians(10)));
        ////            }
        robot.queuer.addDelay(0.5);
        robot.followPPPath(backToStack);
        robot.queuer.addDelay(0.5);
        robot.resetAuto();
        robot.queuer.addDelay(1.5);
        robot.intakeAuto(4-2*i);
        robot.queuer.waitForFinish();
        robot.grabAuto();
        Path stackToBack2 = new Path();
        stackToBack2.add(
            new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        stackToBack2.add(new GeneralWaypoint(-33, -55.5, toRadians(-220), 1.0, 0.4, 8));
        stackToBack2.add(new GeneralWaypoint(16, -55.5, toRadians(-180), 1.0, 0.4, 6));
        stackToBack2.add(new GeneralWaypoint(39, -40, toRadians(-170), 0.9, 0.4, 6));
        stackToBack2.add(new EndWaypoint(46, -35, toRadians(-178), .5, .4, 6, 2, toRadians(10)));
        robot.followPPPath(stackToBack2);
        robot.lowAuto();
        robot.drop();
      }
      robot.resetAuto();
      robot.update();
    }
  }
}
