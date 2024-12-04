package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.ProgBotInitialize;
import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TextFiles.ProgBotInitialize;

import java.io.FileWriter;
import java.util.ArrayList;

    @TeleOp(name = "RecordDriverInputs", group = "Linear Opmode")
    public class captureTele extends LinearOpMode {
        ProgBotInitialize robot;


            private ArrayList<String> recordedInputs;

            // Variable for speed control
            private double speedMultiplier = 1.0;

            @Override
            public void runOpMode() {

                    robot = new ProgBotInitialize(this, false);

                    // InitileftDrivealize hardware


                    recordedInputs = new ArrayList<>();


                    telemetry.addData("Status", "Ready to record driver inputs");
                    telemetry.update();

                    waitForStart();

                    long startTime = System.currentTimeMillis();

                    while (opModeIsActive()&&!gamepad1.share) {

                        // Adjust speed with Cross (X) or Circle
                        if (gamepad1.cross) {
                            speedMultiplier = 1.5; // Speed up
                        } else if (gamepad1.circle) {
                            speedMultiplier = 0.5; // Slow down
                        } else {
                            speedMultiplier = 1.0; // Normal speed
                        }

                        // Get joystick inputs for forward/backward, strafe, and turning
                        double forwardBackward = gamepad1.right_trigger - gamepad1.left_trigger;
                        double strafe = gamepad1.right_stick_x; // Right Stick X for strafe
                        double turn = gamepad1.left_stick_x; // Left Stick X for turning

                        // Apply speed multiplier to inputs
                        double leftPower = (forwardBackward + strafe + turn) * speedMultiplier;
                        double rightPower = (forwardBackward - strafe - turn) * speedMultiplier;

                        // Set power to motors
                        robot.fLeft.setPower(leftPower);
                        robot.bLeft.setPower(leftPower);

                        robot.fRight.setPower(rightPower);
                        robot.bRight.setPower(rightPower);

                        // Record inputs with timestamp
                        recordedInputs.add((System.currentTimeMillis() - startTime) + "," + leftPower + "," + rightPower);

                        telemetry.addData("Recording", "Left Power: %.2f, Right Power: %.2f, Speed Multiplier: %.2f", leftPower, rightPower, speedMultiplier);
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



