package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TextFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.RobotInitialize;

import java.io.FileWriter;
import java.util.ArrayList;

@TeleOp(name = "COPYRecordDriverInputs", group = "Linear Opmode")
public class captureTeleCopy extends LinearOpMode {
    RobotInitialize robot;


        private ArrayList<String> recordedInputs;

        // Variable for speed control
        private double speedMultiplier = 1.0;

        @Override
        public void runOpMode() {

//                robot = new ProgBotInitialize(this, false);
            robot = new RobotInitialize(this, false);


            // InitileftDrivealize hardware


                recordedInputs = new ArrayList<>();


                telemetry.addData("Status", "Ready to record driver inputs");
                telemetry.update();

                waitForStart();

                long startTime = System.currentTimeMillis();


                while (opModeIsActive()&&!gamepad1.share) {

                    // Adjust speed with Cross (X) or Circle

                    double strafeVelocity;
                    double straightMovementVelocity;
                    double turnVelocity;

                    double fleftVel = 0;
                    double bleftVel = 0;
                    double frightVel= 0 ;
                    double brightVel = 0;


                    {
                        int speed = 2700;

                        if (gamepad1.circle) {
                            speed = 270;
                        }
                        if (gamepad1.cross) {
                            speed = 6969;
                        }

                        strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * speed;
                        //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
                        turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * speed;
                        //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
                        straightMovementVelocity = 0;
                        if (gamepad1.left_trigger > 0) {
                            //slow
                            straightMovementVelocity = Math.pow(gamepad1.left_trigger, 3) * speed;
                            //strafeVelocity = 0*(gamepad1.left_trigger);
                            //turnVelocity =0 * (gamepad1.left_trigger);

                        } else if (gamepad1.right_trigger > 0) {
                            //slow
                            straightMovementVelocity = -Math.pow(gamepad1.right_trigger, 3) * speed;
                            //strafeVelocity = 0*(gamepad1.right_trigger);
                            //turnVelocity = 0 *(gamepad1.right_trigger);

                        } else {
                            straightMovementVelocity = 0;
                        }

                        if (gamepad1.left_trigger > 0) {
                            //slow

                            straightMovementVelocity = 2700 * (gamepad1.left_trigger);
                            //strafeVelocity = 0*(gamepad1.left_trigger);
                            //turnVelocity =0 * (gamepad1.left_trigger);

                        }
                        if (gamepad1.right_trigger > 0) {
                            //slow
                            straightMovementVelocity = -2700 * (gamepad1.right_trigger);
                            //strafeVelocity = 0*(gamepad1.right_trigger);
                            //turnVelocity = 0 *(gamepad1.right_trigger);

                        }
                        if (gamepad1.circle) {
                            //slow
                            if (gamepad1.left_trigger > 0) {
                                straightMovementVelocity = 270;
                            } else if (gamepad1.right_trigger > 0) {
                                straightMovementVelocity = -270;

                            }
                            strafeVelocity = 270 * Math.signum(gamepad1.right_stick_x);
                            //turnVelocity = 0 * Math.signum(gamepad1.right_stick_x);
                            if (gamepad1.left_stick_x > 0) {
                                turnVelocity = 270 * Math.signum(gamepad1.left_stick_x);
                            } else if (gamepad1.left_stick_x > 0) {
                                turnVelocity = -270 * Math.signum(gamepad1.left_stick_x);
                            }
                        }
                        if (gamepad1.cross) {
                            //boost
                            if (gamepad1.left_trigger > 0) {
                                straightMovementVelocity = 6969;
                            } else if (gamepad1.right_trigger > 0) {
                                straightMovementVelocity = -6969;

                            }
                            strafeVelocity = 6969 * Math.signum(gamepad1.right_stick_x);

                            if (gamepad1.left_stick_x > 0) {
                                turnVelocity = 6969 * Math.signum(gamepad1.left_stick_x);
                            } else if (gamepad1.left_stick_x > 0) {
                                turnVelocity = -6969 * Math.signum(gamepad1.left_stick_x);
                            }
                        }
                        fleftVel = (strafeVelocity - straightMovementVelocity + turnVelocity);
                        frightVel = (-strafeVelocity - straightMovementVelocity - turnVelocity);
                        bleftVel = (strafeVelocity + straightMovementVelocity - turnVelocity);
                        brightVel = (-strafeVelocity + straightMovementVelocity + turnVelocity);

                        {
                            robot.fLeft.setVelocity(fleftVel); // Overall
                            // negative value
                            robot.fRight.setVelocity(frightVel); // Overall
                            // positive value
                            robot.bLeft.setVelocity(bleftVel); // Overall
                            // positive value
                            robot.bRight.setVelocity(brightVel); // Overall
                            // negative value
                        }
                    }

                    // Record inputs with timestamp
                    recordedInputs.add((System.currentTimeMillis() - startTime) + "," + (fleftVel) + "," + (frightVel) + "," + (bleftVel) + "," + (brightVel));

//                    telemetry.addData("Recording", "fleft Velocity: %.2f, fright Velocity: %.2f, bleft Velocity: %.2f, bright Velocity %.2f,", (startTime),(fleftVel),(frightVel),(bleftVel), (brightVel));
                    telemetry.update();
                }

                // Save recorded inputs after the session
                saveInputsToFile("/sdcard/FIRST/recordedInputsOnce.txt");

        }

        private void saveInputsToFile(String filename) {
            try (FileWriter writer = new FileWriter(filename)) {
                for (String input : recordedInputs) {
                    writer.write(input + "\n");
                }
                telemetry.addData("Status", "Inputs saved for future use");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
            telemetry.update();
        }
    }



