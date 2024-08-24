package com.millburnx.utils

import java.awt.Color
import java.awt.Graphics
import java.awt.Rectangle
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import javax.swing.JPanel
import javax.swing.border.EmptyBorder

/**
 * A custom popover component
 */
class JPopover(
    val parent: JPanel,
    val scale: Double,
    val bgColor: Color = Utils.Colors.bg0,
    val borderColor: Color = Utils.Colors.bg2,
    val borderRadius: Double = 6.0,
    val borderWidth: Float = 1f,
    val padding: Vec2d = Vec2d(16),
    val margin: Vec2d = Vec2d(8),
) : JPanel() {
    private var pos: Vec2d = Vec2d(0)

    init {
        isVisible = false
        background = Color(0, 0, 0, 0)
        border = EmptyBorder(((padding + borderWidth) * scale).insets())
        font = parent.font

        addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) {
                // block click bubbling
            }
        })
    }

    fun show(pos: Vec2d) {
        this.pos = pos
        isVisible = true
        updateBounds()
    }

    fun close() {
        isVisible = false
    }

    fun add() {
        parent.add(this)
        parent.revalidate()
        parent.repaint()
    }

    fun remove() {
        parent.remove(this)
        parent.revalidate()
        parent.repaint()
    }

    fun updateBounds() {
        val pSize = Vec2d(preferredSize)
        val margin = margin * scale
        val size = pSize + margin
        var optimalPos = pos.copy() // top left corner
        optimalPos = if (pos.x + size.x > parent.width) {
            // doesn't fit on the right
            if (pos.x - size.x < 0) {
                // doesn't fit on the left, do center
                Vec2d(pos.x + pSize.x / 2, optimalPos.y)
            } else {
                // fits on the left
                Vec2d(pos.x - size.x, optimalPos.y)
            }
        } else {
            // fits on the right
            Vec2d(pos.x + margin.x, optimalPos.y)
        }
        optimalPos = if (pos.y + size.y > parent.height) {
            // doesn't fit on the bottom
            if (pos.y - size.y < 0) {
                // doesn't fit on the top, do center
                Vec2d(optimalPos.x, pos.y + pSize.y / 2)
            } else {
                // fits on the top
                Vec2d(optimalPos.x, pos.y - size.y)
            }
        } else {
            // fits on the bottom
            Vec2d(optimalPos.x, pos.y + margin.y)
        }
        bounds = Rectangle(optimalPos.awt(), pSize.dimension())
        revalidate()
        repaint()
        parent.revalidate()
        parent.repaint()
    }

    override fun paintComponent(g: Graphics) {
        val (bufferedImage, g2d) = Utils.bufferedImage(width, height)
        Utils.drawRoundedPanel(
            g2d,
            scale,
            Vec2d(width, height),
            bgColor,
            borderRadius,
            borderColor,
            borderWidth,
        )
        super.paintComponent(g2d)
        g.drawImage(bufferedImage, 0, 0, null)
    }
}