package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="potentiometer map test")
public class PotentiometerCalibration extends LinearOpMode {

    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "Analog_Port_0_CH");
        Servo gimbalPos = hardwareMap.get(Servo.class, "Servo_Port_2_CH");

        telemetry.addData("ready", true);
        telemetry.update();

        // Ensure the directory exists
        File directory = new File("/sdcard/FIRST/potentiometerLogs");
        if (!directory.exists()) {
            directory.mkdirs();
        }

        // File writer setup
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("/sdcard/FIRST/potentiometerLogs/potentiometer_data.txt", true));
        } catch (IOException e) {
            telemetry.addData("Error", "File not created");
            telemetry.update();
        }

        waitForStart();
        if (writer != null) {
            try {
                while (opModeIsActive()) {
                    for (double i = 1; i > 0; i -= 0.001) {
                        if (isStopRequested()) break;

                        gimbalPos.setPosition(i);
                        double startTime = clock.milliseconds();
                        while (clock.milliseconds() - startTime < 200) {
                            // Wait 100 ms
                        }

                        double voltage = potentiometer.getVoltage();
                        String data = i + "," + voltage + "\n";

                        // Write to the file and flush the buffer
                        writer.write(data);
                        writer.flush(); // Ensures the data is written immediately


                        while (clock.milliseconds() - startTime < 200) {
                            // Wait 100 ms
                        }

                        telemetry.addData("Position", i);
                        telemetry.addData("Voltage", voltage);
                        telemetry.addData("Time", clock.milliseconds() - startTime);
                        telemetry.update();
                    }

                    telemetry.addData("finished", true);
                    telemetry.update();

                    //Wait until stop is requested
                    while (opModeIsActive()) {
                        if (isStopRequested()) {
                            break;
                        }
                    }
                }
            } catch (IOException e) {
                telemetry.addData("Error", "Writing to file failed");
                telemetry.update();
            } finally {
                try {
                    writer.close();
                } catch (IOException e) {
                    telemetry.addData("Error", "Failed to close the file");
                    telemetry.update();
                }
            }
        }
    }
}
