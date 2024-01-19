package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
public class RedRightAut extends LinearOpMode {
  int bark = 1;

  @Override
  public void runOpMode() throws InterruptedException {
    BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(15.5, -60, Math.toRadians(-90)));
    while (!isStarted() || isStopRequested()) {
      bark = robot.getSpikePos();
      telemetry.addData("pixel", bark);
    }
    while (!isStopRequested() && opModeIsActive() && !robot.queuer.isFullfilled()) {
      // move to spike
      robot.upAuto();
      robot.purpurAuto();
      robot.dropAuto(0);
      // move to backdrop
      robot.lowAuto();
      robot.drop();
//      for (int i = 0; i < 3; i++) {
      // move away
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
