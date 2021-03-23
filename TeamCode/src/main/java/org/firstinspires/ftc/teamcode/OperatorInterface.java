package org.firstinspires.ftc.teamcode;

import com.technototes.control.gamepad.GamepadButton;
import com.technototes.control.gamepad.GamepadStick;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.ConditionalCommand;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.library.control.gamepad.CommandAxis;
import com.technototes.library.control.gamepad.CommandButton;
import com.technototes.library.control.gamepad.CommandGamepad;

import org.firstinspires.ftc.teamcode.commands.drivebase.AlignToShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.NormalSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.SnailSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.TurboSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmExtendCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.index.SendRingToShooterCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootAtRateCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterSetSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleCloseCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleLowerCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleOpenCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleRaiseCommand;
import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

/** Class for driver controls
 *
 */
public class OperatorInterface {
    /** The robot
     *
     */
    public Robot robot;

    /** Gamepads
     *
     */
    public CommandGamepad driverGamepad, codriverGamepad;

    /** Buttons for intake
     * mainbutton toggles between intaking in and off, and spit just extakes
     */
    public CommandButton intakeMainButton, intakeSpitButton;

    public CommandButton wobbleClawButton, wobbleArmButton;

    public CommandButton firePrepButton;
    public CommandAxis fireAxis;

    public CommandButton shooterSpeedPrepButton;
    public CommandAxis shooterSpeedAxis;

    public GamepadStick driveLStick, driveRStick;
    public CommandButton turboModeButton, snailModeButton;

    public OperatorInterface(CommandGamepad driver, CommandGamepad codriver, Robot r){

        //instantiate objects
        driverGamepad = driver;
        codriverGamepad = codriver;
        robot = r;

        //set buttons
        intakeMainButton = driverGamepad.a;
        intakeSpitButton = driverGamepad.b;

        wobbleClawButton = driverGamepad.x;
        wobbleArmButton = driverGamepad.y;

        firePrepButton = driverGamepad.leftBumper;
        fireAxis = driverGamepad.leftTrigger;
        //to actually fire
        fireAxis.setTriggerThreshold(0.1);

        shooterSpeedPrepButton = driverGamepad.rightBumper;
        shooterSpeedAxis = driverGamepad.rightTrigger;

        driveLStick = driverGamepad.leftStick;
        driveRStick = driverGamepad.rightStick;

        turboModeButton = driverGamepad.leftStickButton;
        snailModeButton = driverGamepad.rightStickButton;

        wobbleArmButton.whenToggled(new WobbleRaiseCommand(robot.wobbleSubsystem))
                .whenInverseToggled(new WobbleLowerCommand(robot.wobbleSubsystem));
        wobbleClawButton.whenToggled(new WobbleOpenCommand(robot.wobbleSubsystem))
                .whenInverseToggled(new WobbleCloseCommand(robot.wobbleSubsystem));

        //intake commands
        intakeMainButton.whenToggled(new IntakeInCommand(robot.intakeSubsystem))
                .whenInverseToggled(new IntakeStopCommand(robot.intakeSubsystem));

        intakeSpitButton.whenPressed(new IntakeOutCommand(robot.intakeSubsystem))
                .whenReleased(new IntakeStopCommand(robot.intakeSubsystem));

        shooterSpeedPrepButton.whilePressed(new ShooterSetSpeedCommand(robot.shooterSubsystem, shooterSpeedAxis));
        firePrepButton.whenPressed(new ParallelCommandGroup(
                new IndexPivotUpCommand(robot.indexSubsystem),
                new AlignToShootCommand(robot.drivebaseSubsystem, robot.shooterSubsystem)))
                .schedule(fireAxis, new SequentialCommandGroup(new ArmExtendCommand(robot.indexSubsystem), new WaitCommand(()->1-fireAxis.getAsDouble()), new ArmRetractCommand(robot.indexSubsystem)))
                .whenReleased(new IndexPivotDownCommand(robot.indexSubsystem));
        //drive command
        CommandScheduler.getInstance().scheduleJoystick(new DriveCommand(robot.drivebaseSubsystem, driveLStick, driveRStick), ()->true);

        snailModeButton.whenPressed(new SnailSpeedCommand(robot.drivebaseSubsystem))
                .whenReleased(new NormalSpeedCommand(robot.drivebaseSubsystem));
        turboModeButton.whenPressed(new TurboSpeedCommand(robot.drivebaseSubsystem))
                .whenReleased(new NormalSpeedCommand(robot.drivebaseSubsystem));

    }
}
