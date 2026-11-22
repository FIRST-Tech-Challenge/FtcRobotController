package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Moves one servo at a time so you can find its useful positions (e.g. claw
 * open/closed). Write the numbers down: they go straight into the real code.
 * A servo is not sent any position until you touch it, so nothing jumps at START.
 *
 *   dpad up/down       pick a servo
 *   left stick X       move a normal servo smoothly
 *   dpad left/right    nudge a normal servo by 0.01
 *   Y                  send a normal servo to 0.5
 *   left stick Y       run a continuous-rotation servo
 */
@TeleOp(name = "Servo Tester", group = "Diagnostics")
public class ServoTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        TreeMap<String, HardwareDevice> sorted = new TreeMap<>();
        for (Map.Entry<String, Servo> e : hardwareMap.servo.entrySet()) sorted.put(e.getKey(), e.getValue());
        for (Map.Entry<String, CRServo> e : hardwareMap.crservo.entrySet()) sorted.put(e.getKey(), e.getValue());
        List<String> names = new ArrayList<>(sorted.keySet());
        List<HardwareDevice> servos = new ArrayList<>(sorted.values());

        if (servos.isEmpty()) {
            telemetry.addLine("No servos in the active config.");
            telemetry.update();
            waitForStart();
            return;
        }

        double[] position = new double[servos.size()];
        boolean[] touched = new boolean[servos.size()];
        for (int i = 0; i < position.length; i++) position[i] = 0.5;

        int selected = 0;
        boolean prevUp = false, prevDown = false, prevLeft = false, prevRight = false;

        telemetry.addLine("Press START. Servos stay put until you move them.");
        telemetry.update();
        waitForStart();

        long lastNanos = System.nanoTime();
        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastNanos) / 1e9;
            lastNanos = now;

            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;

            if ((up && !prevUp) || (down && !prevDown)) {
                HardwareDevice old = servos.get(selected);
                if (old instanceof CRServo) ((CRServo) old).setPower(0);
                selected = (selected + (up ? -1 : 1) + servos.size()) % servos.size();
            }
            HardwareDevice device = servos.get(selected);

            telemetry.addData("Selected", names.get(selected));

            if (device instanceof Servo) {
                Servo servo = (Servo) device;
                double before = position[selected];
                double p = before + gamepad1.left_stick_x * 0.5 * dt; // full stick = 0.5/second
                if (left && !prevLeft) p -= 0.01;
                if (right && !prevRight) p += 0.01;
                if (gamepad1.y) p = 0.5;
                p = Range.clip(p, 0, 1);
                if (p != before || gamepad1.y) touched[selected] = true;
                position[selected] = p;
                if (touched[selected]) servo.setPosition(p);

                telemetry.addData("Type", "servo");
                telemetry.addData("Location", HubNames.of(hardwareMap, servo.getController())
                        + " port " + servo.getPortNumber());
                telemetry.addData("Position", touched[selected] ? String.format("%.3f", p) : "not sent yet");
            } else {
                CRServo cr = (CRServo) device;
                double power = -gamepad1.left_stick_y;
                cr.setPower(power);

                telemetry.addData("Type", "continuous rotation");
                telemetry.addData("Location", HubNames.of(hardwareMap, cr.getController())
                        + " port " + cr.getPortNumber());
                telemetry.addData("Power", "%.2f", power);
            }
            prevUp = up; prevDown = down; prevLeft = left; prevRight = right;

            telemetry.addLine();
            for (int i = 0; i < names.size(); i++) {
                String pos = servos.get(i) instanceof Servo && touched[i]
                        ? String.format("  %.3f", position[i]) : "";
                telemetry.addLine((i == selected ? "> " : "  ") + names.get(i) + pos);
            }
            telemetry.update();
        }

        for (HardwareDevice d : servos) {
            if (d instanceof CRServo) ((CRServo) d).setPower(0);
        }
    }
}
