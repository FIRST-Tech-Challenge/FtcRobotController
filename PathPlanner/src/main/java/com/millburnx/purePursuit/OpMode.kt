package com.millburnx.purePursuit

import com.millburnx.purePursuit.Utils.Point
import com.millburnx.purePursuit.ftcDashboard.FTCDashboard
import java.awt.Color

interface IOpMode {
    val telemetry: ITelemetry

    fun init()
    fun loop(): Boolean // return false top stop, true or continue (can't get kotlin to do allow only `return;` without removing typing)
}

interface ITelemetry {
    fun addData(key: String, value: Any)
    fun clear()
    fun update()
}

class DrawData(val color: Color, val size: Double)

abstract class OpMode(ppi: Double, private val updateHertz: Double) : IOpMode {
    // Rendering
    val ftcDashboard = FTCDashboard(ppi)

    // FTC
    class Telemetry : ITelemetry {
        private val data = mutableMapOf<String, Any>()

        override fun addData(key: String, value: Any) {
            println("Telemetry.addData(key=$key, value=$value)")
            data[key] = value
        }

        override fun clear() {
            println("Telemetry.clear()")
            data.clear()
        }

        override fun update() {
            println("Telemetry.update()")
            // draw data
        }
    }

    override val telemetry = Telemetry()

    val robot: Robot = Robot(Point(16.0, 14.0), 14.0)

    fun drive(x: Double, y: Double, rx: Double) {
        robot.drive(x, y, rx)
    }

    fun start() {
        init()
        while (loop()) {
            update()
            if (updateHertz > 0) {
                Thread.sleep((1000 / updateHertz).toLong())
            }
        }
    }

    var lastFrame = System.nanoTime()
    private fun update() {
        val currentFrame = System.nanoTime()
        val deltaTime = (currentFrame - lastFrame) / 1e9
        lastFrame = currentFrame
        robot.update(deltaTime)
    }
}