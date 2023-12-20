package com.example.uitesting.ui

import java.awt.*
import java.awt.image.BufferStrategy

/**
 * Wrapper for the drawing canvas.
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
