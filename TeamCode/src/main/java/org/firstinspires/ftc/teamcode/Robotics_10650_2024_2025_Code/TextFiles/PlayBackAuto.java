package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TextFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.RobotInitialize;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

@Autonomous(name="PlayBackAuto")
public class PlayBackAuto extends LinearOpMode {
    RobotInitialize robot;

    public void runOpMode() {

        // Load the recorded inputs
        ArrayList<String> recordedInputs = loadInputsFromFile("/sdcard/FIRST/recordedInputsOnce.txt");

        telemetry.addData("Status", "Ready to replay inputs");
        telemetry.update();

        waitForStart();
        robot = new RobotInitialize(this, true);

        // Replaying the stored input sequence
        if (recordedInputs != null) {
            telemetry.addData("1", "Ready to replay inputs");
            telemetry.update();


            long startTime = System.currentTimeMillis();

            for (String input : recordedInputs) {
                telemetry.addData("2", "Ready to replay inputs");
                telemetry.update();


                if (!opModeIsActive()) break;
                telemetry.addData("3", "Ready to replay inputs");
                telemetry.update();



                // Parse recorded input
                String[] values = input.split(",");
                long timestamp = Long.parseLong(values[0]);
                double fLeftVelocity = Double.parseDouble(values[1]);
                double fRightVelocity = Double.parseDouble(values[2]);
                double bLeftVelocity = Double.parseDouble(values[3]);
                double bRightVelocity = Double.parseDouble(values[4]);





                // Wait for the right time to replay this input
                while (System.currentTimeMillis() - startTime < timestamp) {
                    idle();
                    telemetry.addData("Current time", System.currentTimeMillis());
                    telemetry.addData("Start time", startTime);
                    telemetry.addData("Timestamp", timestamp);
                    telemetry.update();


                }

                // Set motor powers
                robot.fLeft.setVelocity(fLeftVelocity);
                robot.bLeft.setVelocity(bLeftVelocity);

                robot.fRight.setVelocity(fRightVelocity);
                robot.bRight.setVelocity(bRightVelocity);


                //telemetry.addData("Replaying", "fleft Velocity: %.2f, fright Velocity: %.2f, bleft Velocity: %.2f, bright Velocity %.2f,", strafeVelocity - straightMovementVelocity + turnVelocity, -strafeVelocity - straightMovementVelocity - turnVelocity, (strafeVelocity + straightMovementVelocity - turnVelocity), (-strafeVelocity + straightMovementVelocity + turnVelocity));
               // telemetry.update();
            }
        }

        // Stop motors at the end
        robot.fLeft.setVelocity(0);
        robot.bLeft.setVelocity(0);

        robot.fRight.setVelocity(0);
        robot.bRight.setVelocity(0);
        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }

    private ArrayList<String> loadInputsFromFile(String filename) {
        ArrayList<String> inputs = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            while ((line = reader.readLine()) != null) {
                inputs.add(line);
            }
            telemetry.addData("Status", "Inputs loaded successfully");
        } catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
        telemetry.update();
        return inputs;
    }
}


