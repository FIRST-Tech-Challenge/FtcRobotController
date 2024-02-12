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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
@Disabled
@Autonomous
@Config
public class RedLeftAut extends LinearOpMode {
  int bark = 1;
  public static double x1 = -37.5, y1 = -36, h1 = -180, v1 = 0.5, w1 = 0.1, fR1 = 6,bf1 = 2,
      x2 = -52.5, y2 = -36, h2 = -180, v2 = 0.4, w2 = 0.18, fR2 = 5, bf2 = 1,
      x3 = -52.5, y3 = -34, h3 = -183, v3 = 0.4, w3 = 0.15, fR3 = 4, bf3 = 2,
      xx1 = 10.4, yy1 = -56, hh1 = -160, vv1 = 1.0, ww1 = 0.3, ffR1 = 6,
      xx2 = -28.5, yy2 = -56, hh2 = -180, vv2 = 1.0, ww2 = 0.3, ffR2 = 6,
      xx3 = -49.5, yy3 = -41, hh3 = -200, vv3 = 0.9, ww3 = 0.3, ffR3 = 6;

  @Override
  public void runOpMode() throws InterruptedException {
    BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(-38, -61, Math.toRadians(-90)));
    Path[] toSpike = new Path[3];
    Waypoint start = new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(-38, -61, new Rotation2d(toRadians(-90))));
    toSpike[0] = new Path(start);
    toSpike[0].add(new GeneralWaypoint(-40, -50, toRadians(-70), 0.5, 0.3, 5));
    toSpike[0].add(new EndWaypoint(-41, -36, toRadians(-70), 0.4, 0.2,5, 2, toRadians(10)));
    toSpike[1] = new Path(start);
    toSpike[1].add(new EndWaypoint(-38, -41, toRadians(-91), 0.5, 0, 5, 2, toRadians(10)));
    toSpike[2] = new Path(start);
    toSpike[2].add(new GeneralWaypoint(-38, -46, toRadians(-90), 0.5, 0.3, 5));
    toSpike[2].add(new EndWaypoint(x1, y1, toRadians(h1), v1, w1, fR1, bf1, toRadians(10)));
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
      robot.queuer.addDelay(1.0);
      robot.followPPPath(toSpike[bark]);
      robot.queuer.addDelay(0.2);
      robot.dropAuto(0);
      Path preToStack = new Path();
      //            if(bark==1){
      preToStack.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
      preToStack.add(new EndWaypoint(x2, y2, toRadians(h2), v2, w2, fR2, bf2, toRadians(10)));
      //            }
      robot.queuer.addDelay(0.3);
      robot.followPPPath(preToStack);
      robot.queuer.addDelay(.6);
      robot.resetAuto();
      robot.intakeAuto(5);
      robot.queuer.waitForFinish();
      robot.queuer.queue(false,true);
      robot.grabAuto();
      robot.queuer.addDelay(0.5);
      robot.lowAuto();
      Path stackToBack = new Path();
      //            if(bark==1){
      double y = 0, yy = 0, h=0;
      if (bark == 0) {
        y = -29.5;
        yy = -32;
        h = toRadians(-180);
      }
      if (bark == 1) {
        y = -35.5;
        yy = -38;
        h=toRadians(-180);
      }
      if (bark == 2) {
        y = -41.5;
        yy = -45;
        h=toRadians(-180);
      }

      stackToBack.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
      stackToBack.add(new GeneralWaypoint(-32, -57, toRadians(-200), 1.0, 0.3, 6));
      stackToBack.add(new GeneralWaypoint(10, -57, toRadians(-185), 1.0, 0.3, 6));
      stackToBack.add(new GeneralWaypoint(38, yy, h, 0.9, 0.3, 6));
      stackToBack.add(new EndWaypoint(44.25, y, toRadians(-179), .5, .3, 5, 3, toRadians(5)));
      //            }
      robot.followPPPath(stackToBack);
      robot.drop();
      for (int i = 0; i < 2; i++) {
        Path backToStack = new Path();
        backToStack.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
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
        robot.queuer.queue(false,true);
        robot.grabAuto();
        Path stackToBack2 = new Path();
        stackToBack2.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        stackToBack2.add(new GeneralWaypoint(-32, -57, toRadians(-200), 1.0, 0.3, 6));
        stackToBack2.add(new GeneralWaypoint(10, -57, toRadians(-185), 1.0, 0.3, 6));
        stackToBack2.add(new GeneralWaypoint(38, -45, toRadians(-180), 1.0, 0.3, 6));
        stackToBack2.add(new EndWaypoint(44, -38, toRadians(-179), .5, .3, 5, 1, toRadians(10)));
        robot.followPPPath(stackToBack2);
        robot.lowAuto();
        robot.drop();
      }
      robot.resetAuto();
      robot.update();
    }
  }
}
