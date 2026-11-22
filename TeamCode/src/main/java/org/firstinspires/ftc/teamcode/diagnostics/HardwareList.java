package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Lists every device in the active hardware configuration on the Driver Hub:
 * its config name, what kind of device it is, and which hub and port it is on.
 * Nothing moves. Select it, press INIT and read the screen.
 */
@TeleOp(name = "Hardware List", group = "Diagnostics")
public class HardwareList extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        List<String> lines = new ArrayList<>();
        for (HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            String kind = kindOf(device);
            if (kind == null) continue; // internal device the team never configured by hand
            for (String name : hardwareMap.getNamesOf(device)) {
                lines.add(String.format("%-8s %-18s %s", kind, name, whereIs(device)));
            }
        }
        Collections.sort(lines);

        while (!isStopRequested()) {
            telemetry.addLine("Config: " + activeConfigName());
            telemetry.addLine(lines.size() + " devices (type / name / location)");
            telemetry.addLine();
            for (String line : lines) telemetry.addLine(line);
            telemetry.update();
            sleep(500);
        }
    }

    private String activeConfigName() {
        try {
            return new com.qualcomm.ftccommon.configuration.RobotConfigFileManager()
                    .getActiveConfig().getName();
        } catch (Exception e) {
            return "?";
        }
    }

    /** Short label for the device type, or null to hide it. */
    private static String kindOf(HardwareDevice d) {
        if (d instanceof LynxModule) return "Hub";
        if (d instanceof DcMotor) return "Motor";
        if (d instanceof CRServo) return "CRServo";
        if (d instanceof Servo) return "Servo";
        if (d instanceof IMU) return "IMU";
        if (d instanceof WebcamName) return "Webcam";
        if (d instanceof ColorSensor) return "Color";
        if (d instanceof DistanceSensor) return "Distance";
        if (d instanceof TouchSensor) return "Touch";
        if (d instanceof DigitalChannel) return "Digital";
        if (d instanceof VoltageSensor) return null; // one per hub, always there
        return d.getClass().getSimpleName();
    }

    private String whereIs(HardwareDevice d) {
        if (d instanceof DcMotor) {
            DcMotor m = (DcMotor) d;
            return HubNames.of(hardwareMap, m.getController()) + " p" + m.getPortNumber();
        }
        if (d instanceof CRServo) {
            CRServo s = (CRServo) d;
            return HubNames.of(hardwareMap, s.getController()) + " p" + s.getPortNumber();
        }
        if (d instanceof Servo) {
            Servo s = (Servo) d;
            return HubNames.of(hardwareMap, s.getController()) + " p" + s.getPortNumber();
        }
        if (d instanceof LynxModule) {
            return ((LynxModule) d).isParent() ? "(main hub)" : "(RS485 hub)";
        }
        return d.getConnectionInfo();
    }
}
