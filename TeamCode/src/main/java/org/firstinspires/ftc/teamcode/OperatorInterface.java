package org.firstinspires.ftc.teamcode;

import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.gamepad.CommandButton;
import com.technototes.library.control.gamepad.CommandGamepad;

import org.firstinspires.ftc.teamcode.commands.drivebase.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.index.SendRingToShooterCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeStopCommand;

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

    public CommandButton indexFeedButton;

    public OperatorInterface(CommandGamepad driver, CommandGamepad codriver, Robot r){

        //instantiate objects
        driverGamepad = driver;
        codriverGamepad = codriver;
        robot = r;

        //set buttons
        intakeMainButton = driverGamepad.a;
        intakeSpitButton = driverGamepad.x;

        indexFeedButton = codriverGamepad.a;

        indexFeedButton.whenPressed(new SendRingToShooterCommand(robot.indexSubsystem));

        //intake commands
        intakeMainButton.whenToggled(new IntakeInCommand(robot.intakeSubsystem))
                .whenInverseToggled(new IntakeStopCommand(robot.intakeSubsystem));
        intakeSpitButton.whenPressed(new IntakeOutCommand(robot.intakeSubsystem));

        //drive command
        CommandScheduler.getInstance().scheduleJoystick(new DriveCommand(robot.drivebaseSubsystem, driverGamepad.leftStick, driverGamepad.rightStick), ()->true);
    

    }
}
