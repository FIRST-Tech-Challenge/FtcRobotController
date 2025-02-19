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
      limitPivot = 80; // In Degrees

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
   * Calls all update functions related to mekanism
   */
  public void update() {
    arm.update();
    clip.update();
    grabber.update();
  }


  ElapsedTime clipTimer = new ElapsedTime();
  public int clipStage = 0;
  /**
   * Auto clip sequence
   * <p>
   * Call this function at least once per loop to make sure it updates properly
   * <p>
   * To cancel it, set clipStage to 0
   */
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
     * Clamp onto new clip
     *
     * 3:
     * Drop block into funnel
     *
     * 4:
     * Raise funnel
     *
     * 5:
     * Lower funnel again
     *
     * 6:
     * Turn head to the proper angle
     * Pivot to proper angle
     *
     * 7:
     * Slide down into clip
     *
     * 8:
     * Slide black up again
     *
     * 9:
     * Rotate grabber back
     *
     * 10:
     * Unclamp
     *
     * 11:
     * Slide back down
     * Grabber on
     *
     * 12:
     * Slide back up
     *
     * 13:
     * Pivot at proper angle
     *
     *
     */
    switch (clipStage) {
      case 1:
        clip.unclamp();
        arm.setSlide(300);
        grabber.wrist.setPosition(0.1);

        if (!arm.slide.isBusy()) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      case 2:
        clip.clamp();

        if (clipTimer.seconds() >= 1){
          clipStage++;
        }
        break;

      case 3:
        grabber.setGrabber(-1.0, -1.0);

        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      case 4:

        clip.setFunnel(1.0);

        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      case 5:
        clip.setFunnel(0.0);
        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      case 6:
        grabber.setWrist(0.25);
        break;

      default:
        break;

    }
  }
}
