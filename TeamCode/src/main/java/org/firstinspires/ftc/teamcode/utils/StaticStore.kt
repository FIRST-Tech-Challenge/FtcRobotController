package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils
import kotlin.time.Duration
import kotlin.time.TimeSource

object StaticStore {

    private val timeSource = TimeSource.Monotonic
    var lastOpMode: TimeSource.Monotonic.ValueTimeMark? = null

    fun mark() {
        lastOpMode = timeSource.markNow()
    }

    fun duration() = lastOpMode?.let {timeSource.markNow() - it} ?: Duration.INFINITE
}