// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mekanism {

  LinearOpMode myOp;

  public final int
      limitSlide = 4200,
      limitPivot = 3500;

  public final double
      countsPerDegree = 41.855,
      countsPerInch = 120.88;


  public final Arm arm;
  public final Grabber grabber;
  public final Clip clip;
  private final Telemetry telemetry;


  public Mekanism(LinearOpMode opMode) {

    arm = new Arm(opMode, limitSlide, limitPivot, countsPerDegree, countsPerInch);
    clip = new Clip(opMode);
    grabber = new Grabber(opMode);

    clip.unclamp();

    telemetry = opMode.telemetry;
    myOp = opMode;
  }


  /**
   * Calls all update functions for mekanism
   */
  public void update() {
    arm.update();
    clip.update();
    grabber.update();
  }


  public void autoClip() {
    telemetry.addData("pivot current pos", arm.pivot.getCurrentPosition());
    telemetry.addData("slide current pos", arm.slide.getCurrentPosition());

    if (arm.slide.getCurrentPosition() > 150) {
      clip.unclamp();
      arm.slide.setPower(-1);
      grabber.wrist.setPosition(0.1);

    } else {
      clip.clamp();
      arm.slide.setPower(0);
      if (arm.pivot.getCurrentPosition() < 2200) {
        grabber.wrist.setPosition(0.1);
        arm.pivot.setPower(1);
      } else {
        grabber.wrist.setPosition(0);
        arm.slide.setPower(-1);
        if (arm.pivot.getCurrentPosition() < 2500) {
          arm.pivot.setPower(1);
        } else {
          arm.pivot.setPower(0);
        }
      }
    }
  }
}
