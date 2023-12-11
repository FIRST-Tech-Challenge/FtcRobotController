package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "Test Battery Voltage")
public class TestBatteryVoltage extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime batteryTestTime = new ElapsedTime();
    public final FTCDashboardPackets dbp = new FTCDashboardPackets();
    private boolean shouldStop = false;

    private DcMotor light1, light2, light3, light4;
    private boolean lightsOn = false;

    public void init() {
        light1 = hardwareMap.get(DcMotor.class, "light1");
        light2 = hardwareMap.get(DcMotor.class, "light2");
        light3 = hardwareMap.get(DcMotor.class, "light3");
        light4 = hardwareMap.get(DcMotor.class, "light4");
    }

    @Override
    public void loop() {
        double voltage = getBatteryVoltage();
        dbp.createNewTelePacket();
        runLights();

        if ((runtime.seconds() < VoltageConstants.VOLTAGE_POLL_RATE) || shouldStop) {
            dbp.put("Battery Voltage", String.format(Locale.ENGLISH,
                    "%f s : Current Voltage: %.1f v",
                    batteryTestTime.seconds(), voltage));
            dbp.send(true);
            return;
        }

        runtime.reset();

        try {
            FileWriter myWriter = new FileWriter("batteryVoltage.csv");
            myWriter.append(String.format(Locale.ENGLISH,
                    "%f : %f", batteryTestTime.seconds(), voltage));
            myWriter.close();
            dbp.put("File Writer", "Successfully wrote to the file.");
        } catch (IOException e) {
            dbp.put("File Writer", "An error occurred.");
            e.printStackTrace();
        }

        if (voltage < VoltageConstants.CUT_OFF_VOLTAGE) {
            dbp.put("Battery Voltage",
                    String.format(Locale.ENGLISH,
                            "Battery Voltage reached the cutoff of %.1f v",
                            VoltageConstants.CUT_OFF_VOLTAGE));
            stopLights();
            shouldStop = true;
        }
        dbp.send(false);
    }

    private void runLights() {
        if (!lightsOn) {
            light1.setPower(1);
            light2.setPower(1);
            light3.setPower(1);
            light4.setPower(1);
            lightsOn = true;
        }
    }

    private void stopLights() {
        if (lightsOn) {
            light1.setPower(0);
            light2.setPower(0);
            light3.setPower(0);
            light4.setPower(0);
            lightsOn = false;
        }
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    @Override
    public void stop() {
        shouldStop = true;
    }
}
