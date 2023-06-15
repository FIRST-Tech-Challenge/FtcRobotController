package org.firstinspires.ftc.teamcode.commandBased;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.IncrementRotatorPulse;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.ToggleRotatorPower;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@TeleOp(name = "Rotator Tuning", group = "Linear Opmode")
public class RotatorTuningProgram extends BlackOp {

    @Override
    public void go() {

        //cancel all previous commands
        CommandScheduler.getInstance().reset();

        RotatorSubsystem rotatorSS = new RotatorSubsystem(hardwareMap);

        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);

        MoveRotatorToAngle rotatorBack = new MoveRotatorToAngle(rotatorSS, Constants.ROTATOR_FORWARD);
        MoveRotatorToAngle rotatorFront = new MoveRotatorToAngle(rotatorSS, Constants.ROTATOR_BACK);

        IncrementRotatorPulse lowerSmallUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_SMALL_INCREMENT, true, true);
        IncrementRotatorPulse lowerSmallDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_SMALL_INCREMENT, true, false);
        IncrementRotatorPulse lowerLargeUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_LARGE_INCREMENT, true, true);
        IncrementRotatorPulse lowerLargeDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_LARGE_INCREMENT, true, false);

        IncrementRotatorPulse upperSmallUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_SMALL_INCREMENT, false, true);
        IncrementRotatorPulse upperSmallDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_SMALL_INCREMENT, false, false);
        IncrementRotatorPulse upperLargeUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_LARGE_INCREMENT, false, true);
        IncrementRotatorPulse upperLargeDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_LARGE_INCREMENT, false, false);

        ToggleRotatorPower enable = new ToggleRotatorPower(rotatorSS, true);
        ToggleRotatorPower disable = new ToggleRotatorPower(rotatorSS, false);

        SetRotatorRange defaultRange = new SetRotatorRange(rotatorSS, Constants.CURRENT_RANGE);


        defaultRange.schedule();
        rotatorFront.schedule();

        waitForStart();

        Scheduler.launchOnStart(this, () -> {
            CommandScheduler.getInstance().run();

            driver.dpad_up.onRise(lowerLargeUpIncrement::schedule);
            driver.dpad_down.onRise(lowerLargeDownIncrement::schedule);
            driver.dpad_right.onRise(lowerSmallUpIncrement::schedule);
            driver.dpad_left.onRise(lowerSmallDownIncrement::schedule);

            driver.y.onRise(upperLargeUpIncrement::schedule);
            driver.a.onRise(upperLargeDownIncrement::schedule);
            driver.x.onRise(upperSmallUpIncrement::schedule);
            driver.b.onRise(upperSmallDownIncrement::schedule);

            driver.start.onRise(rotatorFront::schedule);
            driver.back.onRise(rotatorBack::schedule);

            driver.left_bumper.onRise(disable::schedule);
            driver.right_bumper.onRise(enable::schedule);

            mTelemetry().addData("usLower", rotatorSS.getPWMRange()[1]);
            mTelemetry().addData("usUpper", rotatorSS.getPWMRange()[2]);
            mTelemetry().addData("pos", rotatorSS.getPosition());
        });
    }
}
