package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Spins one motor at a time so you can see which config name is which wheel or
 * mechanism, and which way it turns. Put the robot on a box first.
 *
 *   dpad up/down   pick a motor
 *   left stick Y   run it (half power; hold right bumper for full)
 *   A              flip its direction
 *   X              zero its encoder
 */
@TeleOp(name = "Motor Tester", group = "Diagnostics")
public class MotorTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        TreeMap<String, DcMotor> sorted = new TreeMap<>();
        for (Map.Entry<String, DcMotor> e : hardwareMap.dcMotor.entrySet()) {
            sorted.put(e.getKey(), e.getValue());
        }
        List<String> names = new ArrayList<>(sorted.keySet());
        List<DcMotor> motors = new ArrayList<>(sorted.values());

        for (DcMotor m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (motors.isEmpty()) {
            telemetry.addLine("No motors in the active config.");
            telemetry.update();
            waitForStart();
            return;
        }

        int selected = 0;
        boolean prevUp = false, prevDown = false, prevA = false, prevX = false;

        telemetry.addLine("Put the robot on a box, then press START.");
        telemetry.addLine("dpad up/down: pick   left stick: run");
        telemetry.addLine("RB: full power   A: flip direction   X: zero encoder");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean a = gamepad1.a, x = gamepad1.x;

            if ((up && !prevUp) || (down && !prevDown)) {
                motors.get(selected).setPower(0);
                selected = (selected + (up ? -1 : 1) + motors.size()) % motors.size();
            }
            DcMotor motor = motors.get(selected);

            if (a && !prevA) {
                motor.setDirection(motor.getDirection() == DcMotorSimple.Direction.FORWARD
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            if (x && !prevX) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            prevUp = up; prevDown = down; prevA = a; prevX = x;

            double scale = gamepad1.right_bumper ? 1.0 : 0.5;
            double power = -gamepad1.left_stick_y * scale;
            motor.setPower(power);

            telemetry.addData("Selected", names.get(selected));
            telemetry.addData("Location", HubNames.of(hardwareMap, motor.getController())
                    + " port " + motor.getPortNumber());
            telemetry.addData("Direction", motor.getDirection());
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Encoder", motor.getCurrentPosition());
            if (motor instanceof DcMotorEx) {
                telemetry.addData("Velocity", "%.0f ticks/s", ((DcMotorEx) motor).getVelocity());
            }
            telemetry.addLine();
            for (int i = 0; i < names.size(); i++) {
                telemetry.addLine((i == selected ? "> " : "  ") + names.get(i));
            }
            telemetry.update();
        }

        for (DcMotor m : motors) m.setPower(0);
    }
}
