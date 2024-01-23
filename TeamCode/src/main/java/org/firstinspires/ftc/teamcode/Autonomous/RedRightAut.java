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
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class RedRightAut extends LinearOpMode {
  int bark = 1;
  public static double followRad1=5, followRad2 = 5, followRad3 =5, x1 = 24.5, y1 = -44, x2 = 48, y2 = -42, x3 = -35, y3= -36,
          pow1 = 0.6, pow2 = 0.55, pow3 = 1.0, buff1 = 2, buff2 = 3, buff3 = 3,
  followRad4 = 5 ,x4 = 40, y4 = -36, pow4 = 1.0, buff4 = 3;

  @Override
  public void runOpMode() throws InterruptedException {
    BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(17, -63, Math.toRadians(-90)));
    Path[] toSpike = new Path[3];
    Waypoint start = new StartWaypoint( new com.arcrobotics.ftclib.geometry.Pose2d(
            17, -64, new Rotation2d(toRadians(-90))));
    toSpike[0] = new Path(start);
    toSpike[0].add(new EndWaypoint(6.5, -44, toRadians(-70), 1.0, 0, 5, 5, toRadians(10)));
    toSpike[1] = new Path(start);
    toSpike[1].add(new EndWaypoint(16.5, -47, toRadians(-91), 0.7, 0, 5, 2, toRadians(10)));
    toSpike[2] = new Path(start);
    toSpike[2].add(new EndWaypoint(x1, y1, toRadians(-90), pow1, 0.2, followRad1, buff1, toRadians(10)));
    Path[] spikeToBackdrop = new Path[3];
    spikeToBackdrop[2] = new Path();
    spikeToBackdrop[2].add(new StartWaypoint(toSpike[2].get(1).getPose()));
    spikeToBackdrop[2].add(new EndWaypoint(44.5, -42, toRadians(-180), 1.0, 0.4, 5, 3, toRadians(10)));
    Path[] preToStack = new Path[3];
    Path stackToBack = new Path();


    while (!isStarted() || isStopRequested()) {
      bark = robot.getSpikePos();
      telemetry.addData("pixel", bark);
      packet.put("pix", bark);
      robot.update();
    }
    bark=2;
    while (!isStopRequested() && opModeIsActive()) {
      robot.followPPPath(toSpike[bark]);
      robot.upAuto();
      robot.purpurAuto();
      robot.queuer.addDelay(0.9);
      robot.dropAuto(0);
      if (bark == 1) {
        spikeToBackdrop[1] = new Path();
        spikeToBackdrop[1].add(
            new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        spikeToBackdrop[1].add(
            new EndWaypoint(48, -36, toRadians(-179), .55, 0.4, 5, 3, toRadians(5)));
      }
      if (bark ==2){
        spikeToBackdrop[2] = new Path();
        spikeToBackdrop[2].add(
                new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        spikeToBackdrop[2].add(
                new EndWaypoint(x2, y2, toRadians(-179), pow2, 0.4, followRad2, buff2, toRadians(5)));
      }
      robot.followPPPath(spikeToBackdrop[bark]);
      robot.queuer.addDelay(0.6);
      robot.veryLowAuto();
      robot.drop();
      if (bark == 1) {
        preToStack[1] = new Path();
        preToStack[1].add(
                new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        preToStack[1].add(
                new EndWaypoint(45, -36, toRadians(-179), .55, 0.4, 5, 3, toRadians(5)));
      }
      if (bark ==2){
        preToStack[2] = new Path();
        preToStack[2].add(
                new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
        preToStack[2].add(
                new EndWaypoint(x2+2, y2, toRadians(-179), pow2, 0.4, followRad2, buff2, toRadians(5)));
      }

      robot.followPPPath(preToStack[bark]);
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
      robot.queuer.addDelay(0.3);
      robot.resetAuto();
//      robot.intakeAuto(5);
//      robot.queuer.waitForFinish();
//      robot.followPPPath(stackToBack);
//      robot.lowAuto();
//      robot.drop();
//      robot.resetAuto();
      robot.update();
    }
  }
}
