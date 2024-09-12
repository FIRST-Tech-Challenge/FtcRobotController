package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Config
@TeleOp
public class PIDMoveForward extends LinearOpMode {
    private PIDController controller;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    public static double targetDistance = 24.0; 

    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        PIDController controller = new PIDController(P, I, D);
        MultipleTelemetry telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        waitForStart();

        double targetTicks = targetDistance * (Drivetrain.TICKS_PER_REV / (2 * Math.PI * Drivetrain.WHEEL_RADIUS_INCHES * Drivetrain.GEAR_RATIO));
        controller.setSetpoint(targetTicks);

        while (opModeIsActive() && !isStopRequested()) {
            double currentTicks = drivetrain.getPosition();
            double pid = controller.calculate(currentTicks);

            drivetrain.move(pid, 0, 0);

            telemetry.addData("Current Position (ticks)", currentTicks);
            telemetry.addData("Target (ticks)", targetTicks);
            telemetry.addData("Power", pid);
            telemetry.update();
        }

        drivetrain.setPowers(0);
    }
}
