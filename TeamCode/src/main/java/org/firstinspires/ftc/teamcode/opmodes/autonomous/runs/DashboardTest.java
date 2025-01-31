package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

public class DashboardTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return commandFactory.extendSlider().andThen(commandFactory.sleep(1000)).andThen(commandFactory.collapseSlider());
    }
}
