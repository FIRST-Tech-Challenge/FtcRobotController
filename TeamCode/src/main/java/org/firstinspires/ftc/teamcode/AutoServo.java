package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Servo",group = "Servo")
public class Servo extends LinearOpMode {
 ElapsedTime runtime = new ElapsedTime();
// i got rid of public

          Servo rightservo;
          Servo leftservo;
      double servoPosition = 0.0;

    @Override
    public void runOpMode() {
            telemetry.addData("Status", "Intialized");
            telemetry.update();

            rightservo = hardwareMap.get(Servo.class,"rightservo");
            rightservo.setPosition(servoPosition);
            leftservo = hardwareMap.get(Servo.class,"leftservo");
            leftservo.setPosition(servoPosition);

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                    // Opens
            if (gamepad2.left_bumper) {
                rightservo.setPosition(1.0);
                leftservo.setPosition(0.0);
                telemetry.addData("button","leftBumper");
                telemetry.update();
            }
            // closes
            else if (gamepad2.right_bumper) {
                    rightservo.setPosition(0.0);
                    leftservo.setPosition(1.0);
                telemetry.addData("button","rightBumper");
                telemetry.update();
           }
      }


        }




    }

