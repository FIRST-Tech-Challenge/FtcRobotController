package com.millburnx.purePursuit.ftcDashboard

import java.awt.Graphics
import java.awt.GraphicsEnvironment
import java.awt.image.BufferedImage
import javax.swing.JPanel

interface IFTCDashboard {
    fun sendTelemetryPacket(telemetryPacket: ITelemetryPacket)
}

class FTCDashboard(ppi: Double) : IFTCDashboard {
    inner class Panel(val ppi: Double) : JPanel() {
        init {
            val ge = GraphicsEnvironment.getLocalGraphicsEnvironment()
            val fonts = ge.availableFontFamilyNames
            for (i in fonts) {
                println("$i ")
            }
        }

        override fun paintComponent(g: Graphics) {
            println("rendering")
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
            println("done")
        }
    }

    val panel: Panel = Panel(ppi)
    private var currentPacket: ITelemetryPacket = TelemetryPacket()
    override fun sendTelemetryPacket(telemetryPacket: ITelemetryPacket) {
//        TODO("Not yet implemented")
        currentPacket = telemetryPacket
//        SwingUtilities.invokeAndWait { panel.repaint() }
        println("Sent telemetry packet")
        panel.repaint()
    }
}