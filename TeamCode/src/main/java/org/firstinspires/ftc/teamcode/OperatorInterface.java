package org.firstinspires.ftc.teamcode;

import com.technototes.control.gamepad.GamepadStick;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.library.control.gamepad.CommandAxis;
import com.technototes.library.control.gamepad.CommandButton;
import com.technototes.library.control.gamepad.CommandGamepad;

import com.technototes.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.drivebase.AlignToShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.TestSplineCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmExtendCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterSetFlapCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterStopCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleCloseCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleLowerCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleOpenCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleRaiseCommand;

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

    public CommandButton testButton, wobbleArmButton;

    public CommandButton firePrepButton;
    public CommandAxis fireAxis;

    public CommandButton shooterFlapButton;
    public CommandAxis shooterFlapAxis;

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

        testButton = driverGamepad.x;
        wobbleArmButton = driverGamepad.y;

        firePrepButton = driverGamepad.leftBumper;
        fireAxis = driverGamepad.leftTrigger;
        //to actually fire
        fireAxis.setTriggerThreshold(0.1);

        shooterFlapButton = driverGamepad.rightBumper;
        shooterFlapAxis = driverGamepad.rightTrigger;

        driveLStick = driverGamepad.leftStick;
        driveRStick = driverGamepad.rightStick;

        turboModeButton = driverGamepad.leftStickButton;
        snailModeButton = driverGamepad.rightStickButton;

        wobbleArmButton.whenToggled(new ParallelCommandGroup(new WobbleOpenCommand(robot.wobbleSubsystem), new WobbleLowerCommand(robot.wobbleSubsystem)))
                .whenInverseToggled(new SequentialCommandGroup(new WobbleCloseCommand(robot.wobbleSubsystem), new WobbleRaiseCommand(robot.wobbleSubsystem)));

        //testButton.whenPressed(new TestSplineCommand(robot.drivebaseSubsystem));

        //intake commands
        intakeMainButton.whenPressed(new IntakeInCommand(robot.intakeSubsystem));
                //.whenInverseToggled(new IntakeStopCommand(robot.intakeSubsystem));

        intakeSpitButton.whenPressed(new IntakeOutCommand(robot.intakeSubsystem))
                .whenReleased(new IntakeStopCommand(robot.intakeSubsystem));

        shooterFlapButton.whilePressed(new ShooterSetFlapCommand(robot.shooterSubsystem, shooterFlapAxis));
        firePrepButton.whenPressed(new ParallelCommandGroup(
                new IndexPivotUpCommand(robot.indexSubsystem),
                new AlignToShootCommand(robot.drivebaseSubsystem, robot.shooterSubsystem),
                new ShooterSetFlapCommand(robot.shooterSubsystem, ()->0.5),
                new IntakeStopCommand(robot.intakeSubsystem)))
                .schedule(()->fireAxis.getAsBoolean()&&firePrepButton.getAsBoolean(), new SequentialCommandGroup(new ArmExtendCommand(robot.indexSubsystem), new WaitCommand(()->1-fireAxis.getAsDouble()), new ArmRetractCommand(robot.indexSubsystem)))
                .whenReleased(new IndexPivotDownCommand(robot.indexSubsystem))
                .whenReleased(new ShooterStopCommand(robot.shooterSubsystem))
                .whenReleased(new IntakeInCommand(robot.intakeSubsystem));
        //fireAxis.whenReleased(new ArmRetractCommand(robot.indexSubsystem));
        //drive command
        CommandScheduler.getInstance().scheduleJoystick(new DriveCommand(robot.drivebaseSubsystem, driveLStick, driveRStick), ()->true);

//        snailModeButton.whenPressed(new SnailSpeedCommand(robot.drivebaseSubsystem))
//                .whenReleased(new NormalSpeedCommand(robot.drivebaseSubsystem));
//        turboModeButton.whenPressed(new TurboSpeedCommand(robot.drivebaseSubsystem))
//                .whenReleased(new NormalSpeedCommand(robot.drivebaseSubsystem));

    }
}
