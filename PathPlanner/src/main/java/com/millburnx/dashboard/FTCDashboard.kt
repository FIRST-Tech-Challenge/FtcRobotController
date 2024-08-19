package com.millburnx.dashboard

import java.awt.Color
import java.awt.Graphics
import java.awt.GraphicsEnvironment
import java.awt.RenderingHints
import java.awt.image.BufferedImage
import javax.swing.JButton
import javax.swing.JPanel
import kotlin.math.min

interface IFTCDashboard {
    fun sendTelemetryPacket(telemetryPacket: ITelemetryPacket)
}

class FTCDashboard(
    ppi: Double,
    val start: () -> Unit,
    val stop: () -> Unit,
    var reset: () -> Unit,
    var load: () -> Unit
) :
    IFTCDashboard {
    inner class Panel(var ppi: Double) : JPanel() {
        var deltaTime = 0.0

        init {
            val ge = GraphicsEnvironment.getLocalGraphicsEnvironment()
            val fonts = ge.availableFontFamilyNames
            for (i in fonts) {
                println("$i ")
            }
            val buttonRow = JPanel()
            buttonRow.add(customButton("Start") { start() })
            buttonRow.add(customButton("Stop") { stop() })
            buttonRow.add(customButton("Reset") { reset() })
            buttonRow.add(customButton("Load") { load() })
            add(buttonRow)

            addComponentListener(object : java.awt.event.ComponentAdapter() {
                override fun componentResized(e: java.awt.event.ComponentEvent) {
                    val minSize = min(width, height)
                    val newOptimalPPI = minSize / 144.0
                    if (newOptimalPPI != ppi) {
                        ppi = newOptimalPPI
                        repaint()
                    }
                }
            })
        }

        private fun customButton(text: String, action: () -> Unit): JButton {
            val button = JButton(text)
            button.addActionListener {
                action()
            }
            return button
        }

        override fun paintComponent(g: Graphics) {
            super.paintComponent(g)
//            val g2d = graphics as Graphics2D
            val bufferedImage = BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)
            val g2d = bufferedImage.createGraphics()
            g2d.setRenderingHints(RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON))
            g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            g2d.translate(width / 2, height / 2)
            val canvas = currentPacket.fieldOverlay()
            val ops = canvas.getOperations()
            for (op in ops) {
                op.draw(g2d, ppi, this)
            }

            // fps overlay
            val fps = 1.0 / deltaTime
            g2d.color = Color.WHITE
            g2d.drawString("FPS: $fps", -width / 2 + 10, -height / 2 + 20)

            g.drawImage(bufferedImage, 0, 0, null)
        }
    }

    val panel: Panel = Panel(ppi)
    private var currentPacket: ITelemetryPacket = TelemetryPacket()
    override fun sendTelemetryPacket(telemetryPacket: ITelemetryPacket) {
        currentPacket = telemetryPacket
        panel.repaint()
    }
}