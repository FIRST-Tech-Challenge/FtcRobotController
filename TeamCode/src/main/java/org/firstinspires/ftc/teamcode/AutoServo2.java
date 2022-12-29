package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



    @Autonomous(name = "Servo2",group = "Servo")
    public class AutoServo2 extends LinearOpMode {
        ElapsedTime runtime = new ElapsedTime();

        Servo rightservo;
        Servo leftservo;
        double servoPosition = 0.0;
        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Intialized");
            telemetry.update();

            rightservo = hardwareMap.servo.get("rightservo");
            rightservo.setPosition(servoPosition);
            leftservo = hardwareMap.servo.get("leftservo");
            leftservo.setPosition(servoPosition);

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // Opens
                rightservo.setPosition(1.0);
                leftservo.setPosition(0.0);
sleep(500);
                // closes
                rightservo.setPosition(0.0);
                leftservo.setPosition(1.0);

            }
        }
    }









