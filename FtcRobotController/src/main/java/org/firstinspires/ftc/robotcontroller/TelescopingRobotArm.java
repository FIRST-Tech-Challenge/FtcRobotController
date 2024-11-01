package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Telescoping Robot Arm", group = "Linear Opmode")
public class TelescopingRobotArm extends LinearOpMode {
        private Servo baseServo;
        private Servo jointServo;
        private Servo telescopeServo;

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
            // Initialize servos
            baseServo = hardwareMap.get(Servo.class, "baseServo");
            jointServo = hardwareMap.get(Servo.class, "jointServo");
            telescopeServo = hardwareMap.get(Servo.class, "telescopeServo");

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // Control base rotation
                if (gamepad1.left_stick_x > 0.1) {
                    baseServo.setPosition(baseServo.getPosition() + 0.01); // Adjust increment
                } else if (gamepad1.left_stick_x < -0.1) {
                    baseServo.setPosition(baseServo.getPosition() - 0.01);
                }

                // Control arm elevation
                if (gamepad1.left_stick_y > 0.1) {
                    jointServo.setPosition(jointServo.getPosition() + 0.01);
                } else if (gamepad1.left_stick_y < -0.1) {
                    jointServo.setPosition(jointServo.getPosition() - 0.01);
                }

                // Control telescoping action
                if (gamepad1.right_trigger > 0.1) {
                    telescopeServo.setPosition(telescopeServo.getPosition() + 0.01);
                } else if (gamepad1.left_trigger > 0.1) {
                    telescopeServo.setPosition(telescopeServo.getPosition() - 0.01);
                }

                // Optional: Add telemetry to monitor positions
                telemetry.addData("Base Position", baseServo.getPosition());
                telemetry.addData("Joint Position", jointServo.getPosition());
                telemetry.addData("Telescope Position", telescopeServo.getPosition());
                telemetry.update();

                // Yield to allow other processes to run
                idle();
            }
        }
    }

