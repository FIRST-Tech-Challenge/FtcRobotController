package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "Test Battery Voltage")
public class TestBatteryVoltage extends RobotOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime batteryTestTime = new ElapsedTime();
    private boolean shouldStop = false;
    DcMotor arm;
    @Override
    public void init() {
        super.init();

        telemetry.addData("Arm Zero Position: ", String.valueOf(armZeroPosition));

        moveArm(0.1f, armZeroPosition - 90);
    }

    @Override
    public void robotloop() {
        moveRobot(1, 0, 0);
        double voltage = getBatteryVoltage();
        dbp.createNewTelePacket();

        if ((runtime.seconds() < VOLTAGE_CONSTANTS.VOLTAGE_POLL_RATE) || shouldStop) {
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

        if (voltage < VOLTAGE_CONSTANTS.CUT_OFF_VOLTAGE) {
            dbp.put("Battery Voltage",
                    String.format(Locale.ENGLISH,
                            "Battery Voltage reached the cutoff of %.1f v",
                            VOLTAGE_CONSTANTS.CUT_OFF_VOLTAGE));
        }
        dbp.send(false);
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
