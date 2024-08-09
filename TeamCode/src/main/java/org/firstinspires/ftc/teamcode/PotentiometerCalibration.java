package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="potentiometer map test")
public class PotentiometerCalibration extends LinearOpMode {

    BufferedWriter writer = null;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "Analog_Port_0_CH");
        Servo gimbalPos = hardwareMap.get(Servo.class, "Servo_Port_2_CH");

        telemetry.addData("ready", true);
        telemetry.update();

        // Ensure the directory exists
        File directory = new File("/sdcard/potentiometerLogs");
        if (!directory.exists()) {
            directory.mkdirs();
        }

        waitForStart();

        while (opModeIsActive()) {
            setup();
            for (double i = 1; i > 0; i -= 0.001) {
                if (isStopRequested()) break;

                gimbalPos.setPosition(i);
                double startTime = clock.milliseconds();
                while (clock.milliseconds() - startTime < 100) {
                    // Wait 200 ms
                }

                double voltage = potentiometer.getVoltage();
                String data = i + "," + voltage + "\n";

                // Write to the file and flush the buffer
                try {
                    writer.write(data);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                try {
                    writer.flush(); // Ensures the data is written immediately
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }

                telemetry.addData("Position", i);
                telemetry.addData("Voltage", voltage);
                telemetry.addData("Time", clock.milliseconds() - startTime);
                telemetry.update();
            }
            try {
                writer.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            //Wait until stop is requested
        }
        telemetry.addData("finished", true);
        telemetry.update();
    }
    private void setup() {
        // Get the battery voltage and format it
        double batteryVoltage = getBatteryVoltage();
        String formattedVoltage = String.format("%.2f", batteryVoltage);

        // Create the file name with the battery voltage

        String fileName = "/sdcard/potentiometerLogs/potentiometer_data_" + formattedVoltage + "V.txt";

        // File writer setup
        try {
            writer = new BufferedWriter(new FileWriter(fileName, true));
        } catch (IOException e) {
            telemetry.addData("Error", "File not created");
            telemetry.update();
        }
    }
    // Helper method to get the battery voltage
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
