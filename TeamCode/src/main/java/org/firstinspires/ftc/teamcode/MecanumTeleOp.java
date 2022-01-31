// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MecanumTeleOp", group = "16481")
public class MecanumTeleOp extends LinearOpMode {
   double y, x, rx ;
   double denominator;
   double armPower;
   int arm_position=1;
   double asdflkj = 0.932;
   robot16481 robot = new robot16481();

   final double clawClose = 0.58;
   final double clawOpen = 0.41;

   @Override
   public void runOpMode() throws InterruptedException {
      robot.telemetry = telemetry;
      robot.hwMap = hardwareMap;
      robot.hardwareSetup();

      double tb_position = robot.tbServo.getPosition();
      waitForStart();

      if (isStopRequested()) return;

      double pos=0;
      while (opModeIsActive()) {

         /****************************************************************
          Drive Code
          ***********************************/
         y = -gamepad1.left_stick_y; // Remember, this is reversed!
         x = gamepad1.left_stick_x * 0.98; // Counteract imperfect strafing
         rx = gamepad1.right_stick_x; //increase speed by 2

         // Driver the robot
         robot.drive(x,y,rx);

         /****************************************************************
          ARM Code
          ***********************************/
         // move ARM up/down and turn the table
         robot.moveARM(gamepad2.left_stick_y / 3);
         robot.tbServo.setPosition(gamepad2.right_stick_x / 2 + 0.5);

         // Move the ARM to one level down if not already at the lowest level-1
         if(gamepad2.dpad_down ) {
            arm_position = arm_position > 1 ? arm_position - 1:arm_position;
            robot.setArm(arm_position);
         }
         // Move the ARM to one level up if not already at the topmost level-3
         if(gamepad2.dpad_up )  {
            arm_position = arm_position < 3 ? arm_position + 1:arm_position;
            robot.setArm(arm_position);
         }

         /****************************************************************
              CLAW Code
          ***********************************/
         // Auto claw close when detectect shipping element
         if ((robot.clawRange.getDistance(DistanceUnit.MM) < 100.00 && robot.CLAW_RANGE_ENABLE)
            && (!gamepad2.left_bumper))        {
            robot.clawServo.setPosition(clawClose);
         }

         if (gamepad1.a)
         {
            robot.CLAW_RANGE_ENABLE = !robot.CLAW_RANGE_ENABLE;
         }

         if(gamepad2.left_bumper)  robot.setClawPosition(robot.CLAW_OPEN);
         if(gamepad2.right_bumper) robot.setClawPosition(robot.CLAW_CLOSE);

         /****************************************************************
              Carurosel Code
          ***********************************/
         // To run carurosel wheel when the gamepad-2 trigger is kept pressed.
         // When not pressed stop the motor
         if (gamepad2.left_trigger==1)  { robot.carousel(0.75 , 1); }
         else if (gamepad2.right_trigger==1) { robot.carousel(0.75, -1); }
         else { robot.carousel(0,1); }

         telemetry.addData("# front left", robot.motorFrontLeft.getPower());
         telemetry.addData("# front right", robot.motorFrontRight.getPower());
         telemetry.addData("# back left", robot.motorBackLeft.getPower());
         telemetry.addData("# back right", robot.motorBackRight.getPower());
         telemetry.addData("X:", gamepad1.left_stick_x);
         telemetry.addData("Y:", gamepad1.left_stick_y);
         telemetry.addData("RX:", gamepad1.right_stick_x);

         telemetry.addData("# ARM Power:", armPower);
         telemetry.addData("pot Value", robot.potMeter.getVoltage());
         telemetry.addData("Servo Value: ", robot.clawServo.getPosition());
         telemetry.addData("game pade right x value", gamepad2.right_stick_x);
         telemetry.addData("tb servo postion", robot.tbServo.getPosition());
         telemetry.addData("postion value", pos);

         telemetry.update();

      }
   }
}