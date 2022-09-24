package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially
 *                                    x/square is the front left,
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left       Front Right
 *        ___  '.      ( A )     /|       Wheel             Wheel
 *      .'    '.    '-._____.-'  .'        (X)               (B)
 *     |       |                 |
 *      '.___.' '.               |                Rear
 *               '.             /                 Wheel
 *                \.          .'                   (A)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@Disabled
@Config
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / - Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / - Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / - Rear</font>");
            telemetry.addLine();

            if(gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0);
                telemetry.addLine("Running Motor: Left");
            } else if(gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER);
                telemetry.addLine("Running Motor: Right");
            } else if(gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0);
                telemetry.addLine("Running Motor: Rear");
            } else {
                drive.setMotorPowers(0, 0, 0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
