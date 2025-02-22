// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mekanism {

  LinearOpMode myOp;

  public final int limitSlide = 4200, limitPivot = 80; // In Degrees

  public final double countsPerDegree = 41.855, countsPerInch = 120.88;


  public final Arm arm;
  public final Grabber grabber;
  public final Clip clip;
  private final Telemetry telemetry;


  public Mekanism(LinearOpMode opMode) {
    arm = new Arm(opMode);
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
     * Unclamp to let new clip slide into place
     * Move slide to 150
     * Wrist to 0.0
     *
     * 2:
     * Clamp onto new clip
     *
     * 3:
     * Drop block into funnel
     *
     * 4:
     * Raise funnel to hook block onto clip
     *
     * 5:
     * Lower funnel again
     *
     * 6:
     * Turn head to the proper angle
     * Pivot to proper angle
     *
     * 7:
     * Ram grabber head into the clip
     *
     * 8:
     * Slide black up again
     *
     * 9:
     * Rotate grabber back down to get ready to pick up the assembly
     *
     * 10:
     * Unclamp
     *
     * 11:
     * Slide back down
     * Grabber on
     *
     * 12:
     * Slide back up once more
     *
     * 13:
     * Pivot at proper angle
     *
     *
     */
    switch (clipStage) {

      /// Unclamp to let new clip slide into place
      /// Move slide to 150
      /// Wrist to 0.0
      case 1:
        clip.unclamp();
        arm.setSlide(300);
        grabber.wrist.setPosition(0.1);

        if (!arm.slide.isBusy()) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      /// Clamp onto new clip
      case 2:
        clip.clamp();

        if (clipTimer.seconds() >= 1) {
          clipStage++;
        }
        break;

      /// Drop block into funnel
      case 3:
        grabber.setGrabber(-1.0, -1.0);

        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;


      /// Raise funnel to hook block onto clip
      case 4:

        clip.setFunnel(1.0);

        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      /// Lower funnel again
      case 5:
        clip.setFunnel(0.0);
        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      /// Turn head to the proper angle
      /// Pivot to proper angle
      case 6:
        grabber.setWrist(0.25);
        if (clipTimer.seconds() >= 1) {
          clipStage++;
        }
        break;

      /// Ram grabber head into the clip
      case 7:
        arm.setSlide(50);
        if (!arm.pivot.isBusy()) {
          clipStage++;
        }
        break;

      /// Slide black up again
      case 8:
        arm.setSlide(250);
        if (!arm.pivot.isBusy()) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      /// Rotate grabber back down to get ready to pick up the assembly
      case 9:
        grabber.setWrist(0.0);
        if (clipTimer.seconds() >= 1) {
          clipStage++;
          clipTimer.reset();
        }
        break;

      /// Unclamp
      case 10:
        clip.unclamp();
        if (clipTimer.seconds() >= 1) {
          clipStage++;
        }
        break;

      ///Slide back down
      /// Grabber on
      case 11:
        arm.setSlide(50);
        grabber.setGrabber(1.0, 1.0);
        if (!arm.slide.isBusy()) {
          clipStage++;
          grabber.setGrabber(0.0, 0.0);
        }
        break;

      /// Slide back up once more
      case 12:
        arm.setSlide(300);
        if (!arm.pivot.isBusy()) {
          clipStage++;
        }

        /// Pivot at proper angle
      case 13:
        arm.setPivot(1000);
        if (!arm.pivot.isBusy()) {
          clipStage = 0;
        }
        break;

      default:
        break;
    }
  }


  ElapsedTime autoClipTimer = new ElapsedTime();

  /**
   * Function from Jim
   */
  public void autoClipOld() {

    autoClipTimer.reset();
    clip.clamp();
    clip.setFunnel(0.3);// horizontal funnel position
    // drops into flat funnel
    arm.setSlide(750);// move to drop
    // set to drop angle
    arm.setPivot(1300);
    grabber.setWrist(.08);// position head to drop
    myOp.sleep(500);
    // out take drop sample
    grabber.setGrabber(-0.5, -0.4);

    myOp.sleep(1000);
    grabber.setGrabber(0.0, 0.0);
    arm.setSlide(700);// get out of the way of funnel
    clip.setFunnel(0.0);
    myOp.sleep(100);
    clip.setFunnel(0.35);// raise slowly
    myOp.sleep(600);
    clip.setFunnel(0.47);
    myOp.sleep(500);
    clip.setFunnel(0.57);
    myOp.sleep(200);
    clip.setFunnel(0.2);// back down it should be held by clip
    arm.setSlide(850);
    clip.setFunnel(.2);// wiggle funnel
    myOp.sleep(300);
    clip.setFunnel(0.0);
    myOp.sleep(400);
    arm.setPivot(1450);
    myOp.sleep(400);
    grabber.setWrist(.8);// move wrist to push position
    myOp.sleep(400);
    // seat the sample ramming speed
    //setSlide(10);// seat sample
    myOp.sleep(1000);
    arm.setSlide(500);//move up
    clip.unclamp();
    //- - - - - pivot to pick up
    arm.setPivot(1000);
    myOp.sleep(500);
    grabber.setWrist(.1);// move wrist to pickup position near horz
    // move pivot to pickup
    myOp.sleep(1000);
    // lower to better pick up
    arm.setSlide(250);
    grabber.setGrabber(1.0, 1.0);
    myOp.sleep(1000);
    grabber.setGrabber(0.0, 0.0);
    myOp.sleep(1000);
    // raise to get ready to put on bar
    arm.setPivot(500);
    //setSlide(1100);
    grabber.setWrist(.5);// pickup position
    myOp.sleep(1000);
    clip.clamp();// clamp to ride
  }
}
