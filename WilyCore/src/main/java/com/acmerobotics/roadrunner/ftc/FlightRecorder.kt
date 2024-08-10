package com.acmerobotics.roadrunner.ftc

object FlightRecorder  {
    @JvmStatic
    fun write(ch: String, o: Any) {

    }
}

class DownsampledWriter(val channel: String, val maxPeriod: Long) {
    private var nextWriteTimestamp = 0L
    fun write(msg: Any) {
        val now = System.nanoTime()
        if (now >= nextWriteTimestamp) {
            nextWriteTimestamp = (now / maxPeriod + 1) * maxPeriod
            FlightRecorder.write(channel, msg)
        }
    }
}
