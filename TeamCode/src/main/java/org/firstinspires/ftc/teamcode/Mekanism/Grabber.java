// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

  LinearOpMode myOp;

  public Servo
      intake1,
      intake2,
      wrist;

  private double
      intake1Power = 0,
      intake2Power = 0,
      wristPos = 0;

  public Grabber(LinearOpMode opMode) {

    // Servos for intake
    intake1 = opMode.hardwareMap.get(Servo.class, "intake");
    intake2 = opMode.hardwareMap.get(Servo.class, "intake2");
    wrist = opMode.hardwareMap.get(Servo.class, "wrist");

    intake1.setDirection(FORWARD);
    intake2.setDirection(REVERSE);
    wrist.setDirection(REVERSE);

    wrist.scaleRange(0.65, 1.0);

    myOp = opMode;
  }


  /**
   * Update intake and wrist positions
   */
  public void update() {
    intake1.setPosition((intake1Power + 1) / 2);
    intake2.setPosition((intake2Power + 1) / 2);

    wrist.setPosition(wristPos);
  }

  /**
   * Initial position of the wrist
   */
  public void initWrist() {
    wrist.setPosition(0.65);
  }


  /**
   * Sets the speed of the grabber wheels
   * <br>
   * 1 full intake<br>
   * -1 full outtake
   *
   * @param power1 (-1) to 1
   */
  public void setGrabber(double power1, double power2) {
    intake1Power = power1;
    intake2Power = power2;
  }


  /**
   * Sets the angle of the wrist<br>
   * 0 - Fully down
   * 1 - Fully up
   *
   * @param pos 0 - 1
   */
  public void setWrist(double pos) {
    wristPos = pos;
  }
}
