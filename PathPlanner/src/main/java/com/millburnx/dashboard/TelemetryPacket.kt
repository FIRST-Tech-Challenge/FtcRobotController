package com.millburnx.dashboard

// https://acmerobotics.github.io/ftc-dashboard/javadoc/com/acmerobotics/dashboard/telemetry/TelemetryPacket.html
interface ITelemetryPacket {
    fun addLine(line: String)
    fun put(key: String, value: Any)
    fun putAll(map: Map<String, Any>)
    fun clearLines()
    fun addTimestamp(): Long
    fun fieldOverlay(): ICanvas
}

class TelemetryPacket : ITelemetryPacket {
    private val data = mutableMapOf<String, Any>()
    private val lines = mutableListOf<String>()
    internal val timestamp: Long? = null
    private val canvas: ICanvas = Canvas()

    override fun addLine(line: String) {
        lines.add(line)
    }

    override fun put(key: String, value: Any) {
        data[key] = value
    }

    override fun putAll(map: Map<String, Any>) {
        data.putAll(map)
    }

    override fun clearLines() {
        lines.clear()
    }

    override fun addTimestamp(): Long {
        return System.currentTimeMillis()
    }

    override fun fieldOverlay(): ICanvas {
        return canvas
    }
}