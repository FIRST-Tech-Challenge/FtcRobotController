package com.millburnx.purepursuit

import com.millburnx.utils.Vec2d
import com.millburnx.dashboard.FTCDashboard
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
    var running = false
    // Rendering
    val ftcDashboard = FTCDashboard(ppi, {
        if (!running) {
            Thread {
                lastFrame = 0L
                start()
            }.start()
        }
    }, { stop() }, {}, {})

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

    val robot: Robot = Robot(Vec2d(16.0, 14.0), 14.0)

    fun drive(x: Double, y: Double, rx: Double) {
        robot.drive(x, y, rx)
    }

    fun start() {
        init()
        running = true
        while (loop()) {
            if (!running) {
                break
            }
            update()
            if (updateHertz > 0) {
                Thread.sleep((1000 / updateHertz).toLong())
            }
        }
        running = false
    }

    fun stop() {
        running = false
    }

    var lastFrame = 0L
    private fun update() {
        val currentFrame = System.nanoTime()
        var deltaTime = (currentFrame - lastFrame) / 1e9
        if (lastFrame == 0L) {
            deltaTime = 0.0
        }
        lastFrame = currentFrame
        robot.update(deltaTime)
//        println("Robot: ${robot.position}, ${robot.heading} | $deltaTime $lastFrame")
    }
}