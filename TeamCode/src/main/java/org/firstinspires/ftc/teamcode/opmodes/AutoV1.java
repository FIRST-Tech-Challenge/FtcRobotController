package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.Robot;
import com.technototes.library.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.AlignToShootCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmExtendCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.index.IndexPivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterSetFlapCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterStopCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleCloseCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleLowerCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleOpenCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleRaiseCommand;

/** Main OpMode
 *
 */
@Autonomous(name = "AutoV1")
public class AutoV1 extends CommandOpMode implements Loggable {
    /** The robot
     *
     */
    public Robot robot;
    @Override
    public void uponInit() {
        robot = new Robot();

    }

    @Override
    public void uponStart() {
        Command shoot = new SequentialCommandGroup(new ArmExtendCommand(robot.indexSubsystem), new ArmRetractCommand(robot.indexSubsystem));
        Command shoot3 = new SequentialCommandGroup(shoot, shoot, shoot);
        Command preptoshoot = new ParallelCommandGroup(
                new IndexPivotUpCommand(robot.indexSubsystem),
                new AlignToShootCommand(robot.drivebaseSubsystem, robot.shooterSubsystem),
                new ShooterSetFlapCommand(robot.shooterSubsystem, ()->0.5),
                new IntakeStopCommand(robot.intakeSubsystem));
        Command closeWobble = new ParallelCommandGroup(new WobbleCloseCommand(robot.wobbleSubsystem), new WobbleRaiseCommand(robot.wobbleSubsystem));
        Command openWobble = new SequentialCommandGroup(new WobbleOpenCommand(robot.wobbleSubsystem), new WobbleLowerCommand(robot.wobbleSubsystem));

        Command start = new Command();
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                closeWobble,
                new WaitCommand(1),
                preptoshoot,
                new WaitCommand(5),
                shoot3,
                new WaitCommand(1),
                new ParallelCommandGroup(new IndexPivotDownCommand(robot.indexSubsystem),
                        new ShooterStopCommand(robot.shooterSubsystem),
                        new IntakeInCommand(robot.intakeSubsystem)),
                new SplineCommand(robot.drivebaseSubsystem, 110, 24, 0),
                new WaitCommand(1),
                openWobble,
                new WaitCommand(1),
                new TurnCommand(robot.drivebaseSubsystem, 90),
                new WaitCommand(1),
                new StrafeCommand(robot.drivebaseSubsystem, 50, 0)
        ));
    }
}
