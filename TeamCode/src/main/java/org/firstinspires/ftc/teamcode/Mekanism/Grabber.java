// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

  LinearOpMode myOp;

  Servo
      intake1,
      intake2,
      wrist;

  private double
      intakePower = 0,
      wristPos = 0;

  public Grabber(LinearOpMode opMode) {

    // Servos for intake
    intake1 = opMode.hardwareMap.get(Servo.class, "intake");
    intake2 = opMode.hardwareMap.get(Servo.class, "intake2");
    wrist = opMode.hardwareMap.get(Servo.class, "wrist");

    intake1.setDirection(REVERSE);
    intake2.setDirection(FORWARD);
    wrist.setDirection(REVERSE);

    wrist.scaleRange(0.65, 1.0);

    myOp = opMode;
  }


  /**
   * Update intake and wrist positions
   */
  public void update() {
    intake1.setPosition((intakePower + 1) / 2);
    intake2.setPosition((intakePower + 1) / 2);

    wrist.setPosition(wristPos);
  }


  public void initWrist() {
    wrist.setPosition(0.65);
  }


  /**
   * Sets the speed of the grabber wheels
   * <br>
   * 1 full intake<br>
   * -1 full outtake
   *
   * @param power (-1) to 1
   */
  public void setGrabber(double power) {
    intakePower = power;
  }


  /**
   * Sets the angle of the wrist<br>
   * 0 is fully down
   *
   * @param pos 0 - 1
   */
  public void setWrist(double pos) {
    wristPos = pos;
  }
}
