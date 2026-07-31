package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DualMotorFlywheelShooter;

@Disabled
@TeleOp()
public class FlywheelShooterTuning extends OpMode {

    private DualMotorFlywheelShooter shooter;
    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        shooter = new DualMotorFlywheelShooter(hardwareMap);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Tuning instructions:");
        telemetry.addData("kS", "Set kP and kV to zero and TUNING_VELOCITY to 1. Completely stop the flywheel. Increase kS until flywheel barely moves.");
        telemetry.addData("kV", "Set kP to 1 and TUNING_VELOCITY to maximum velocity the shooter is going to reach. Increase kV until the flywheel's velocity matches the desired velocity.");
        telemetry.addData("kP", "Pick two arbitrary velocities, preferably a high and a low one. Alternate between them and observe the FtcDashboard graph. Increase kP until the oscillation is too jerky or the battery voltage drops too much during the alternation.");
    }

    @Override
    public void loop() {
        shooter.tune();
        shooter.debug(dashboardTelemetry);
    }
}
