package com.millburnx.pathplanner

import com.millburnx.utils.Vec2d
import java.awt.Dimension
import java.awt.event.ComponentAdapter
import java.awt.event.ComponentEvent
import javax.swing.JFrame
import kotlin.math.min


fun main(args: Array<String>) {

    val scale = args.find { it.startsWith("--scale=") }?.substring(8)?.toDoubleOrNull() ?: 1.0
    println("Program Scale: $scale")
    val defaultSize = 1080
    val pathPlanner = PathPlanner(0.0, scale)

    System.setProperty("awt.useSystemAAFontSettings", "on");
    System.setProperty("awt.useSystemAAFontSettings", "lcd");
    System.setProperty("swing.aatext", "true");
    System.setProperty("sun.java2d.xrender", "true");

    val frame = JFrame("Pure Pursuit")
    frame.size = Dimension(defaultSize, defaultSize)
    frame.layout = null
    frame.addComponentListener(object : ComponentAdapter() {
        override fun componentResized(e: ComponentEvent) {
            val width = frame.width
            val height = frame.height
            val size = min(width, height)
            pathPlanner.ppi = size / (144.0 / scale) / scale
            pathPlanner.size = Dimension(size, size)
            pathPlanner.location = ((Vec2d(width, height) - Vec2d(size, size)) / 2.0).awt()
//            pathPlanner.updateButtonBounds()
            println("X: ${pathPlanner.location.x}, Y: ${pathPlanner.location.y}")
            println("PPI: ${pathPlanner.ppi}")
        }
    })
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.setLocationRelativeTo(null)
    frame.contentPane.add(pathPlanner)
    frame.isVisible = true
}
