package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.VoltageConstants;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Locale;

@Autonomous(name = "Test Battery Voltage")
public class TestBatteryVoltage extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime batteryTestTime = new ElapsedTime();
    private ElapsedTime recordingTime = new ElapsedTime();

    public final FTCDashboardPackets dbp = new FTCDashboardPackets();
    private double batteryVoltage = 0;
    private double startingBatteryVoltage = 0;
    private boolean shouldStop = false;

    private DcMotor light1, light2, light3, light4;
    private boolean lightsOn = false;
    private HashMap<Double, Double> storedVoltage;

    public void init() {
        light1 = hardwareMap.get(DcMotor.class, "light1");
        light2 = hardwareMap.get(DcMotor.class, "light2");
        light3 = hardwareMap.get(DcMotor.class, "light3");
        light4 = hardwareMap.get(DcMotor.class, "light4");
        startingBatteryVoltage = getBatteryVoltage();

        if (startingBatteryVoltage < VoltageConstants.getMinimumStartingVoltage()) {
            telemetry.speak("WARNING: Testing the battery with less " +
                    "than the minimum recommended starting voltage...");
        }
    }

    @Override
    public void loop() {
        batteryVoltage = getBatteryVoltage();
        dbp.createNewTelePacket();
        runLights();

        if (minimumBatteryVoltageReached()) {
            dbp.put("Battery Voltage",
                    String.format(Locale.ENGLISH,
                            "Battery Voltage reached the cutoff of %.1f v",
                            VoltageConstants.getCutOffVoltage()));
            dbp.info(String.format(Locale.ENGLISH, "Completed testing in %f seconds",
                    batteryTestTime.seconds()), true);
            dbp.info(String.format(Locale.ENGLISH, "Average Rate of Change: %f",
                    getAverageRateOfChange()), true);
            stopLights();
            shouldStop = true;
        }

        if ((runtime.seconds() < VoltageConstants.getVoltagePollRate()) || shouldStop) {
            dbp.put("Battery Voltage", String.format(Locale.ENGLISH, "%f s",
                    batteryTestTime.seconds()));
            dbp.put("Current Voltage", String.format(Locale.ENGLISH,"%f v", batteryVoltage));
            dbp.send(false);

            if (recordingTime.seconds() >= VoltageConstants.getVoltageRecordRate()) {
                storedVoltage.put(runtime.seconds(), getBatteryVoltage());
                recordingTime.reset();
            }
            return;
        }

        runtime.reset();

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

    boolean minimumBatteryVoltageReached() {
        final Double[] STORED_VOLTAGE_VALUES = storedVoltage.values().toArray(new Double[0]);
        final int STORED_VOLTAGE_VALUES_LENGTH = STORED_VOLTAGE_VALUES.length - 1;
        final double LAST_STORED_VOLTAGE = STORED_VOLTAGE_VALUES[STORED_VOLTAGE_VALUES_LENGTH];

        if (batteryTestTime.seconds() < VoltageConstants.getGracePeriodBeforeCutoff())
            return false;

        final double VOLTAGE_LEEWAY = VoltageConstants.getEndVoltageLeeway();
        return (batteryVoltage < VoltageConstants.getCutOffVoltage())
                || (LAST_STORED_VOLTAGE > (startingBatteryVoltage - VOLTAGE_LEEWAY)
                || (LAST_STORED_VOLTAGE > 12.0d));
    }

    double getAverageRateOfChange() {
        final Double[] STORED_VOLTAGE_VALUES = storedVoltage.values().toArray(new Double[0]);
        final int STORED_VOLTAGE_VALUES_LENGTH = STORED_VOLTAGE_VALUES.length;

        final Double[] STORED_TIME_VALUES = storedVoltage.keySet().toArray(new Double[0]);

        double summedAverages = 0;
        int iterations = 0;

        for (int i = 0; i < STORED_VOLTAGE_VALUES_LENGTH; i++) {
            double voltageLeft = STORED_VOLTAGE_VALUES[i];
            double voltageRight;
            double timeLeft = STORED_TIME_VALUES[i];
            double timeRight;

            try {
                voltageRight = STORED_VOLTAGE_VALUES[i + 1];
                timeRight = STORED_TIME_VALUES[i + 1];
            } catch (IndexOutOfBoundsException e) {
                continue;
            }

            summedAverages += ((timeRight - timeLeft) / (voltageRight - voltageLeft));
            iterations += 1;
        }

        return summedAverages / (double) iterations;
    }

    @Override
    public void stop() {
        shouldStop = true;
    }
}
