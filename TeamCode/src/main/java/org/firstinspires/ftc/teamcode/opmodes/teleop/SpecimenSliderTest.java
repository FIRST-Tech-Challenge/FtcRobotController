package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class SpecimenSliderTest extends OpModeTemplate {

    private TeleDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        SpecimenSlider slider = new SpecimenSlider(hardwareMap, telemetry, null);

        new Trigger(() -> gamepad2.right_stick_y > 0.5)
                .whenActive(new InstantCommand(slider::Expand, slider))
                .whenInactive(new InstantCommand(slider::Hold, slider));

        new Trigger(() -> gamepad2.right_stick_y < -0.5)
                .whenActive(new InstantCommand(slider::Collapse, slider))
                .whenInactive(new InstantCommand(slider::Hold, slider));


        // Register all subsystems
        register(slider);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }
}