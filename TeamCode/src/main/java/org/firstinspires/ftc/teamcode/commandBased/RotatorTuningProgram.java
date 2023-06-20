package org.firstinspires.ftc.teamcode.commandBased;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.IncrementRotatorPulse;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.ToggleRotatorPower;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
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

        ArmSubsystem armSS = new ArmSubsystem(hardwareMap);
        RotatorSubsystem rotatorSS = new RotatorSubsystem(hardwareMap);

        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);

        MoveArmToAngle positionArm = new MoveArmToAngle(armSS, Constants.ARM_ANGLE_FRONT);

        MoveRotatorToPosition rotatorBack = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_BACK);
        MoveRotatorToPosition rotatorFront = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_FRONT);

        IncrementRotatorPulse lowerSmallUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_SMALL_INCREMENT, true);
        IncrementRotatorPulse lowerSmallDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_SMALL_INCREMENT, true);
        IncrementRotatorPulse lowerLargeUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_LARGE_INCREMENT, true);
        IncrementRotatorPulse lowerLargeDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_LARGE_INCREMENT, true);

        IncrementRotatorPulse upperSmallUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_SMALL_INCREMENT, false);
        IncrementRotatorPulse upperSmallDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_SMALL_INCREMENT, false);
        IncrementRotatorPulse upperLargeUpIncrement = new IncrementRotatorPulse(rotatorSS, Constants.ROTATOR_LARGE_INCREMENT, false);
        IncrementRotatorPulse upperLargeDownIncrement = new IncrementRotatorPulse(rotatorSS, -Constants.ROTATOR_LARGE_INCREMENT, false);

        ToggleRotatorPower enable = new ToggleRotatorPower(rotatorSS, true);
        ToggleRotatorPower disable = new ToggleRotatorPower(rotatorSS, false);

        SetRotatorRange defaultRange = new SetRotatorRange(rotatorSS, Constants.TUNED_RANGE);


        positionArm.schedule();
        defaultRange.schedule();
        rotatorFront.schedule();

        waitForStart();

        Scheduler.launchOnStart(this, () -> {
            CommandScheduler.getInstance().run();

            driver.left_bumper.onRise(defaultRange::schedule);

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

            mTelemetry().update();
        });
    }
}
