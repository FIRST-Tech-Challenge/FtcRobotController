package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.delivery.Hang;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class OdometryTest extends OpModeTemplate {

    private TeleFourWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, null);
        DeliverySlider slider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, null);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        AutoMecanumDriveTrain driveTrain = new AutoMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, null);

        rollingIntake.SetElbowInIntakePosition();

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(slider::CollapseMinInAuto, slider));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(slider::ExtendMaxInAuto, slider));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(deliveryPivot::MoveToIntakeInAuto, deliveryPivot));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(deliveryPivot::MoveToDeliveryInAuto, deliveryPivot));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(deliveryPivot::MoveToStartInAuto, deliveryPivot));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                    new InstantCommand(rollingIntake::Intake, rollingIntake),
                    new InstantCommand(deliveryPivot::MoveToIntakeSampleInAuto, deliveryPivot)
                ));

        // Register all subsystems
        register(driveTrain, slider, rollingIntake, deliveryPivot);
        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    public void IntakeSleep() {
        sleep(1000);
    }

    private void switchToMode(PowerMode powerMode) {
        driveTrain.setPowerRatio(powerMode.getPowerRatio());
    }
}