package org.firstinspires.ftc.teamcode.components.easytoggle

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.DataSupplier

class EasyToggles(
    val up: EasyToggle = EasyToggle(),
    val down: EasyToggle = EasyToggle(),
    val _in: EasyToggle = EasyToggle(),
    val out: EasyToggle = EasyToggle(),
) {
    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<EasyToggle>) {
        telemetry.addData("up", dataSupplier(up))
        telemetry.addData("down", dataSupplier(down))
        telemetry.addData("in", dataSupplier(_in))
        telemetry.addData("out", dataSupplier(out))
    }
}
