package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Mec::Diagnostics", group="MechBot")
public class MechDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot2 createRobot() {
        return new ToboMech().configureLogging("ToboMech", LOG_LEVEL);
    }
}
