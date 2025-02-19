// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Clip {

  LinearOpMode myOp;

  private final Servo
      clip1,
      clip2,
      funnel;

  private double
      funnelPos = 0;

  private boolean clamped = false;


  public Clip(LinearOpMode opMode) {

    clip1 = opMode.hardwareMap.get(Servo.class, "ramp 1");
    clip2 = opMode.hardwareMap.get(Servo.class, "ramp 2");
    funnel = opMode.hardwareMap.get(Servo.class, "funnel");

    clip1.setDirection(FORWARD);
    clip2.setDirection(REVERSE);
    funnel.setDirection(FORWARD);

    clip1.scaleRange(0, 0.15);
    clip2.scaleRange(0, 0.15);
    funnel.scaleRange(0, 0.035);
    myOp = opMode;
  }


  /**
   * Updates the position of everything related to the clipping mechanism
   */
  public void update() {

    if (clamped) {
      clip1.setPosition(1);
      clip2.setPosition(1);
    } else {
      clip1.setPosition(0);
      clip2.setPosition(0);
    }

    funnel.setPosition(funnelPos);
  }


  /**
   * Sets the angle of the clip funnel<br>
   * 0 - flat
   * 1 - fully up
   *
   * @param pos 0 - 1
   */
  public void setFunnel(double pos) {
    funnelPos = pos;
  }


  /**
   * Clamp onto the clip
   */
  public void clamp() {
    clamped = true;
  }


  /**
   * Unclamp the clip
   */
  public void unclamp() {
    clamped = false;
  }
}
