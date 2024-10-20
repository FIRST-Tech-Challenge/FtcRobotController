package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.DriveChassis;
import org.firstinspires.ftc.teamcode.util.Numbers;

@TeleOp(name="Xendy's Teleop", group = "z_group")
public class XendysTestTeleop extends OpMode {
    private XendyChassis chassis;
    private double x, y, r, targetAngle;

    @Override
    public void init() {
        chassis = new XendyChassis(this);
    }

    @Override
    public void loop() {
        // Gather rotational data
        double radYaw = chassis.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double degYaw = Numbers.normalizeAngle(chassis.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // Handle Inputs
        lerpInput();

        // Calculate Rotation
        targetAngle += r * 3;
        targetAngle = Numbers.normalizeAngle(targetAngle);
        double rp = getTurnCorrection(degYaw, targetAngle);

        // Calculate movement
        double rotX = x * Math.cos(-radYaw) - y * Math.sin(-radYaw);
        double rotY = x * Math.sin(-radYaw) + y * Math.cos(-radYaw);
        rotX *= 1.1;

        // Move motors
        double mult = 100;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rp), 1);
        chassis.leftFrontMotor.setVelocity(((rotY + rotX + rp) / denominator) * mult);
        chassis.leftBackMotor.setVelocity(((rotY - rotX + rp) / denominator) * mult);
        chassis.rightFrontMotor.setVelocity(((rotY - rotX - rp) / denominator) * mult);
        chassis.rightBackMotor.setVelocity(((rotY + rotX - rp) / denominator) * mult);

        // Telemetry
        telemetry.addData("Yaw", degYaw);
        telemetry.addData("Target", targetAngle);
        telemetry.addData("Rotation POwer", rp);
        telemetry.update();
    }

    public double getTurnCorrection(double current, double target) {
        double left = Numbers.normalizeAngle(target - current);
        double right = Numbers.normalizeAngle(current - target);
        double angle = Math.abs(left) < Math.abs(right) ? -left : right;
        if (Math.abs(angle) < 0.5) return 0;
        return Range.clip(angle / 90, -1, 1);
    }

    public static final double LERP_DIFF_CANCEL = 0.05;
    public static final double LERP_MULT = 0.2;
    public void lerpInput() {
        double inputX = gamepad1.left_stick_x;
        double inputY = gamepad1.left_stick_y;
        double inputR = gamepad1.right_stick_x;

        double diffX = inputX - x;
        double diffY = inputY - y;
        double diffR = inputR - r;
        if (Math.abs(diffX) < LERP_DIFF_CANCEL) x = inputX;
        else x += diffX * LERP_MULT;
        if (Math.abs(diffY) < LERP_DIFF_CANCEL) y = inputY;
        else y += diffY * LERP_MULT;
        if (Math.abs(diffR) < LERP_DIFF_CANCEL) r = inputR;
        else r += diffR * LERP_MULT;
    }
}
