// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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


  int clipStage = 0;


  /**
   * Auto clip sequence
   */
  ElapsedTime clipTimer = new ElapsedTime();
  public void autoClip() {
    telemetry.addData("pivot current pos", arm.pivot.getCurrentPosition());
    telemetry.addData("slide current pos", arm.slide.getCurrentPosition());


    /*
     * Runs through the stages of the autocliping sequence
     * Each case sets what happens, while checking if all movements have been completed to see if it can move on
     *
     * 1:
     * Unclamp
     * Move slide to 150
     * Wrist to 0.1
     *
     * 2:
     * Drop block into funnel
     *
     * 3:
     * Raise funnel
     */
    switch (clipStage) {
      case 1:
        clip.unclamp();
        arm.setSlide(300);
        grabber.wrist.setPosition(0.1);

        if (!arm.slide.isBusy()) {
          clipStage++;
        }
        break;

      case 2:
        grabber.setGrabber(-1);

        if (!arm.slide.isBusy()) {
          clipStage++;
        }
        break;

      default:
        break;

    }

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
