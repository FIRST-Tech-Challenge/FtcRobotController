package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw extends LinearOpMode {

  //init the two servos that control the claw
  private Servo left = null;
  private Servo right = null;

  //these positions, between 0 and 1, were determined through testing
  private double max_left_pos = 0.4;
  private double min_left_pos = 0.25;
  private double max_right_pos = .58;
  private double min_right_pos = .65;
  
  // private double max_left_pos = 0;
  // private double min_left_pos = .25;
  // private double max_right_pos = 1;
  // private double min_right_pos = .75;

  //constructor
  public Claw (Servo s, Servo s2) {
    left = s;
    right = s2;
  }

  //two functions, one sets the two to the grab pos, the other to the release pos
  //all you need is to tell the servo to go to a certain position.
  //the servos will automatically slew to the correct position.
  public void grab() {
    left.setPosition(min_left_pos);
    right.setPosition(min_right_pos);
  }
  public void release() {
    left.setPosition(max_left_pos);
    right.setPosition(max_right_pos);
  }

  public void runOpMode() {

  }
}
