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
  public static double followRad1=10, followRad2 = 20;

  @Override
  public void runOpMode() throws InterruptedException {
    BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(15.5, -60, Math.toRadians(-90)));
    Path[] toSpike = new Path[3];
    Waypoint start = new StartWaypoint( new com.arcrobotics.ftclib.geometry.Pose2d(
            15.5, -60, new Rotation2d(toRadians(-90))));
    toSpike[1] = new Path(start);
    toSpike[1].add(new EndWaypoint(12.5, -40, toRadians(-90), 1.0, 0.2, followRad1, 1.5, toRadians(10)));
    toSpike[2] = new Path(start);
    toSpike[2].add(new EndWaypoint(22.5, -48, toRadians(-90), 0.9, .4, 10, .5, toRadians(5)));
    Path[] spikeToBackdrop = new Path[3];
    spikeToBackdrop[1] = new Path();
    spikeToBackdrop[1].add(new StartWaypoint(toSpike[1].get(1).getPose()));
    spikeToBackdrop[1].add(new EndWaypoint(37.5, -30, toRadians(-180), 1.0, 0.7, followRad2, 1.5, toRadians(10)));
    spikeToBackdrop[2] = new Path();
    spikeToBackdrop[2].add(new StartWaypoint(toSpike[2].get(1).getPose()));
    spikeToBackdrop[2].add(new EndWaypoint(34.5, -40, toRadians(-180), 1.0, 0.7, 20, .5, toRadians(10)));
    while (!isStarted() || isStopRequested()) {
      bark = robot.getSpikePos();
      telemetry.addData("pixel", bark);
      packet.put("pix", bark);
      robot.update();
    }
    bark=1;
    while (!isStopRequested() && opModeIsActive() && !robot.queuer.isFullfilled()) {
      robot.followPPPath(toSpike[bark], true);
//      robot.upAuto();
//      robot.queuer.addDelay(1.5);
//      robot.purpurAuto();
      robot.queuer.addDelay(0.8);
      robot.dropAuto(0);
      robot.followPPPath(spikeToBackdrop[bark], true);
//      robot.lowAuto();
      robot.drop();
//      for (int i = 0; i < 3; i++) {
      // move away
      robot.queuer.addDelay(5);
      robot.resetAuto();
//        robot.queuer.addDelay(2.5);
//        robot.intakeAuto(4);
//        // move to backdrop
//        robot.lowAuto();
//        robot.drop();
//      }
//      //move to park
//      robot.resetAuto();
      robot.update();
    }
  }
}
