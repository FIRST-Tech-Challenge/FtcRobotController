package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.PowerMode;
import org.firstinspires.ftc.teamcode.subsystems.delivery.Hang;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleDriveTrain;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class HangTest extends OpModeTemplate {

    private TeleDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        Hang hang = new Hang(hardwareMap, driverGamepad, null, null);

        // OPERATOR Actions

        // Delivery Pivot
        new Trigger(() -> gamepad2.right_stick_y > 0.5)
                .whenActive(new InstantCommand(hang::Expand, hang))
                .whenInactive(new InstantCommand(hang::Hold, hang));

        new Trigger(() -> gamepad2.right_stick_y < -0.5)
                .whenActive(new InstantCommand(hang::Collapse, hang))
                .whenInactive(new InstantCommand(hang::Hold, hang));


        // Register all subsystems
        register(hang);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
    }

    private void switchToNitroMode() {
        switchToMode(PowerMode.NITRO);
    }

    private void switchToRegularMode() {
        switchToMode(PowerMode.REGULAR);
    }

    private void switchToSlowMode() {
        switchToMode(PowerMode.SLOW);
    }

    private void switchToMode(PowerMode powerMode) {
        driveTrain.setPowerRatio(powerMode.getPowerRatio());
    }
}