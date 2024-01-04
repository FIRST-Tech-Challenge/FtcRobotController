package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class BTCommand extends CommandBase{
    Command withInterrupt(BooleanSupplier condition) {
        return until(condition);
    }
    Command until(BooleanSupplier condition) {
        return raceWith(new WaitUntilCommand(condition));
    }
    RepeatCommand repeatedly() {
        return new RepeatCommand(this);
    }



}