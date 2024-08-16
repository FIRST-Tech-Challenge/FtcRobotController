package com.millburnx.purePursuit.ftcDashboard

import java.awt.Graphics
import java.awt.GraphicsEnvironment
import java.awt.image.BufferedImage
import javax.swing.JButton
import javax.swing.JPanel

interface IFTCDashboard {
    fun sendTelemetryPacket(telemetryPacket: ITelemetryPacket)
}

class FTCDashboard(ppi: Double, val start: () -> Unit, val stop: () -> Unit, var reset: () -> Unit, var load: () -> Unit) : IFTCDashboard {
    inner class Panel(val ppi: Double) : JPanel() {
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
            g2d.translate(width / 2, height / 2)
            val canvas = currentPacket.fieldOverlay()
            val ops = canvas.getOperations()
            for (op in ops) {
                op.draw(g2d, ppi)
            }
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