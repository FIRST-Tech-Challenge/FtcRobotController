package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Competition TeleOp. Everything is on gamepad 1.
 *
 *   left stick      drive and strafe (field-centric)
 *   right stick x   turn
 *
 *   right bumper    index forward, feeds balls to the shooter
 *   left bumper     index backwards, clears a jam
 *   right trigger   intake
 *   left trigger    outtake
 *   B / circle      shooter at far speed
 *   A / cross       shooter at close speed
 *
 * All six mechanism controls are hold-to-run: the motor spins while you hold the button and stops
 * the moment you let go. Shooting and feeding are separate, so to fire you hold B (or A) to get the
 * wheels up to speed, then hold the right bumper to feed a ball into them.
 *
 * Y, X and the D-pad are deliberately left unbound - they are reserved for the Limelight controls.
 */
@TeleOp(name = "Comp TeleOp", group = "Drive")
public class CompTeleOp extends CommandOpMode {

    private DrivebaseSubsystem drivebase;
    private IntakeSubsystem intake;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;

    @Override
    public void initialize() {
        // Clear anything left over from a previous OpMode run before registering new subsystems.
        CommandScheduler.getInstance().reset();

        drivebase = new DrivebaseSubsystem(hardwareMap);
        intake    = new IntakeSubsystem(hardwareMap);
        index     = new IndexSubsystem(hardwareMap);
        shooter   = new ShooterSubsystem(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);

        // Driving runs continuously in the background whenever nothing else claims the drivebase.
        drivebase.setDefaultCommand(new DriveCommand(drivebase, driver));

        // --- Index: right bumper feeds, left bumper clears ---
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new StartEndCommand(index::forward, index::stop, index));

        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new StartEndCommand(index::reverse, index::stop, index));

        // --- Intake: triggers are analog, so they need a threshold rather than a button press ---
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                > RobotConstants.TRIGGER_THRESHOLD)
                .whileActiveOnce(new StartEndCommand(intake::intake, intake::stop, intake));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                > RobotConstants.TRIGGER_THRESHOLD)
                .whileActiveOnce(new StartEndCommand(intake::outtake, intake::stop, intake));

        // --- Shooter: B is the far shot, A is the close shot ---
        new GamepadButton(driver, GamepadKeys.Button.B)
                .whenHeld(new StartEndCommand(shooter::shootFar, shooter::stop, shooter));

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whenHeld(new StartEndCommand(shooter::shootClose, shooter::stop, shooter));

        telemetry.addLine("Ready.");
        telemetry.addLine("Point the robot away from the driver station before pressing START -");
        telemetry.addLine("that heading becomes 'forward' and there is no reset button.");
        telemetry.update();
    }

    /**
     * Same loop as {@link CommandOpMode}, with two additions: the heading is zeroed the instant the
     * match starts (so the robot can be re-aimed between INIT and START), and telemetry is updated
     * every cycle.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        drivebase.resetYaw();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            telemetry.addData("Heading", "%.1f deg", drivebase.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Intake (M1)", "%.2f", intake.getPower());
            telemetry.addData("Index (M2)", "%.2f", index.getPower());
            telemetry.addData("Shooter (M3/M4)", "%.2f", shooter.getPower());
            telemetry.addData("Shooter ticks/sec", "%.0f / %.0f",
                    shooter.getLeftVelocity(), shooter.getRightVelocity());
            telemetry.update();
        }

        reset();
    }
}
