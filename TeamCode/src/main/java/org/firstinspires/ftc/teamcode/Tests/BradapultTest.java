package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous
@Config
public class BradapultTest extends RFServoTest {
  public static double LOAD = 0.0, SHOOT = 1.0;
  public static int target = 0;
  private int atTarg = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    initialize("launcherServo", 0.5, LOAD, SHOOT);
    waitForStart();
    while (opModeIsActive()) {
      double[] pussitions = {LOAD, SHOOT};

      if (target != atTarg) {
        flipTo(pussitions[target]);
        atTarg = target;
      }
      packet.put("atTarg", atTarg);
      robot.update();
    }
  }
}
