package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class wristTest extends LinearOpMode {

  Servo wrist;
  double set;
  int lastButton;

  @Override
  public void runOpMode() throws InterruptedException {
    initRobot();
    waitForStart();
    while(opModeIsActive()){

      // sets booleans
      if (gamepad1.a) {
        if (lastButton == -1) {
          set += .05;
          lastButton = 1;
        }
      } else if (gamepad1.b) {
        if (lastButton == -1) {
          set -= .05;
          lastButton = 2;
        }
      } else lastButton = -1;
      wrist.setPosition(set);
      telemetry.addData("gamepad: ",-gamepad1.left_stick_y);
      telemetry.addData("servo: ", wrist.getPosition());
      telemetry.update();
    }
  }

  public void initRobot(){
    wrist = hardwareMap.get(Servo.class,"wrist");
    wrist.setDirection(Servo.Direction.FORWARD);
    set = 0;
    lastButton = -1;
  }
}
