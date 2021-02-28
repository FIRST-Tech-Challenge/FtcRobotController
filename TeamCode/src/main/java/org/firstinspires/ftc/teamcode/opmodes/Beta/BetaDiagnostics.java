package org.firstinspires.ftc.teamcode.opmodes.Beta;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Beta.ToboBeta;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Diag-Beta", group="Beta")
public class BetaDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot2 createRobot() {
        return new ToboBeta().configureLogging("ToboBeta", LOG_LEVEL);
    }
}
