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
import java.util.Locale;

@Autonomous(name = "Test Battery Voltage")
public class TestBatteryVoltage extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime batteryTestTime = new ElapsedTime();
    public final FTCDashboardPackets dbp = new FTCDashboardPackets();
    private double batteryVoltage = 0;
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
        batteryVoltage = getBatteryVoltage();
        dbp.createNewTelePacket();
        runLights();

        if ((runtime.seconds() < VoltageConstants.VOLTAGE_POLL_RATE) || shouldStop) {
            dbp.put("Battery Voltage", String.format(Locale.ENGLISH, "%f s",
                    batteryTestTime.seconds()));
            dbp.put("Current Voltage", String.format(Locale.ENGLISH,"%f v", batteryVoltage));
            dbp.send(true);
            return;
        }

        runtime.reset();

        if (batteryVoltage < VoltageConstants.CUT_OFF_VOLTAGE) {
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
