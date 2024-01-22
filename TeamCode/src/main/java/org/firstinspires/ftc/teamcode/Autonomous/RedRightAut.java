package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
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
  public static double followRad1=5, followRad2 = 6, followRad3 =5, x1 = 16.5, y1 = -46, x2 = 44.5, y2 = -37, x3 = -35, y3= -36,
          pow1 = 1.0, pow2 = 1.0, pow3 = 1.0, buff1 = 4, buff2 = 3, buff3 = 3,
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
    toSpike[1].add(new EndWaypoint(x1, y1, toRadians(-91), pow1, 0, followRad1, buff1, toRadians(10)));
    toSpike[2] = new Path(start);
    toSpike[2].add(new EndWaypoint(22.5, -48, toRadians(-90), 1.0, 0, 5, 3, toRadians(5)));
    Path[] spikeToBackdrop = new Path[3];
    spikeToBackdrop[1] = new Path();
    spikeToBackdrop[1].add(new StartWaypoint(toSpike[1].get(1).getPose()));
    spikeToBackdrop[1].add(new EndWaypoint(x2, y2, toRadians(-179), pow2, 0.4, followRad2, buff2, toRadians(5)));
    spikeToBackdrop[2] = new Path();
    spikeToBackdrop[2].add(new StartWaypoint(toSpike[2].get(1).getPose()));
    spikeToBackdrop[2].add(new EndWaypoint(34.5, -40, toRadians(-180), 1.0, 0.4, 5, 3, toRadians(10)));
    Path[] preToStack = new Path[3];
    preToStack[1] = new Path();
    preToStack[1].add(new StartWaypoint(spikeToBackdrop[1].get(1).getPose()));
    preToStack[1].add(new EndWaypoint(x3, y3, toRadians(-180), pow3, 0.3, followRad3, buff3, toRadians(5)));
    Path stackToBack = new Path();
    stackToBack.add(new StartWaypoint(preToStack[1].get(preToStack[1].size()-1).getPose()));
    stackToBack.add(new EndWaypoint(x4, y4, toRadians(-180), pow4, 0.3, followRad4, buff4, toRadians(5)));

    while (!isStarted() || isStopRequested()) {
      bark = robot.getSpikePos();
      telemetry.addData("pixel", bark);
      packet.put("pix", bark);
      robot.update();
    }
    bark=1;
    while (!isStopRequested() && opModeIsActive()) {
      robot.followPPPath(toSpike[bark]);
      robot.upAuto();
      robot.purpurAuto();
      robot.queuer.addDelay(0.9);
      robot.dropAuto(0);
      robot.followPPPath(spikeToBackdrop[bark]);
      robot.queuer.addDelay(0.6);
      robot.lowAuto();
      robot.drop();
//      robot.followPPPath(preToStack[bark]);
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
