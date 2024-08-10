package com.wilyworks.simulator.framework

import java.awt.Canvas
import java.awt.Dimension
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import java.awt.image.BufferStrategy
import javax.swing.BoxLayout
import javax.swing.JFrame
import javax.swing.JPanel
import kotlin.system.exitProcess

/**
 * Wrapper for the Telemetry window frame.
 */
class TelemetryWindow(title: String, width: Int, height: Int) : JFrame() {
    var internalWidth = width
    var internalHeight = height

    val canvas = MainCanvas(internalWidth, internalHeight)
    val canvasPanel = JPanel()

    init {
        setTitle(title)

        defaultCloseOperation = DO_NOTHING_ON_CLOSE
        addWindowListener(object : WindowAdapter() {
            override fun windowClosing(we: WindowEvent?) {
                super.windowClosing(we)

                dispose()
                exitProcess(0)
            }
        })

        setSize(internalWidth, internalHeight)
        setLocation(0, 0)

        isResizable = false

        layout = BoxLayout(contentPane, BoxLayout.X_AXIS)

        canvasPanel.layout = BoxLayout(canvasPanel, BoxLayout.Y_AXIS)
        canvasPanel.add(canvas)

        contentPane.add(canvasPanel)
        pack()

        canvas.start()
    }
}

/**
 * Wrapper for the Telemetry drawing canvas.
 */
class MainCanvas(private var internalWidth: Int, private var internalHeight: Int): Canvas() {
    lateinit var bufferStrat: BufferStrategy

    init {
        setBounds(0, 0, internalWidth, internalHeight)
        preferredSize = Dimension(internalWidth, internalHeight)
        ignoreRepaint = true
    }

    fun start() {
        createBufferStrategy(2)
        bufferStrat = bufferStrategy

        requestFocus()
    }

    override fun getPreferredSize(): Dimension {
        return Dimension(internalWidth, internalHeight)
    }
}
