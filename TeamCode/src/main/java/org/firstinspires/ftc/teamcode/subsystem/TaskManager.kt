package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class TaskManager : Subsystem() {
    class Call(
        val elapsedTime: ElapsedTime
    ) {
        val init: MutableSet<(ElapsedTime) -> Unit> = mutableSetOf()
        val action: MutableSet<(ElapsedTime) -> Unit> = mutableSetOf()
        operator fun invoke() {
            for (function in init) {
                function(elapsedTime)
            }
            for (function in action) {
                function(elapsedTime)
            }
        }
    }
    val elapsedTime: ElapsedTime = ElapsedTime()
    val initLoop: Call = Call(elapsedTime)
    val loop: Call = Call(elapsedTime)

    override fun printState(telemetry: Telemetry) {
        TODO("Not yet implemented")
    }
}
