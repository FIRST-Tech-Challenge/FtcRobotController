package com.millburnx.pathplanner

import com.millburnx.utils.Bezier
import com.millburnx.utils.Vec2d
import java.awt.Color
import java.awt.Graphics
import java.awt.RenderingHints
import java.awt.event.KeyAdapter
import java.awt.event.KeyEvent
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.image.BufferedImage
import javax.swing.JPanel

class PathPlanner(val ppi: Double) : JPanel() {
    init {
        background = Color(0x0e1a25)

        // the bezier point and the actual point (anchor, prevHandle, nextHandle)
        var selectedBezier: Pair<BezierPoint, BezierPoint.PointType>? = null

        addMouseListener(object : MouseAdapter() {
            override fun mousePressed(e: MouseEvent) {
                if (e.button != MouseEvent.BUTTON1) return

                val clickPoint = Vec2d(e.x, e.y).minus(Vec2d(width / 2, height / 2)).div(ppi)
                for (point in points) {
                    for (type in BezierPoint.PointType.entries) {
                        val threshold = when (type) {
                            BezierPoint.PointType.ANCHOR -> 20 / ppi
                            else -> 15 / ppi
                        }
                        val distance = point.getType(type)?.distanceTo(clickPoint) ?: Double.POSITIVE_INFINITY
                        if (distance < threshold) {
                            selectedBezier = Pair(point, type)
                            return
                        }
                    }
                }

                points.add(BezierPoint(clickPoint))
                addState()
                repaint()
            }

            override fun mouseReleased(e: MouseEvent?) {
                if (selectedBezier != null) {
                    addState()
                }
                selectedBezier = null
            }
        })

        addMouseMotionListener(object : MouseAdapter() {
            override fun mouseDragged(e: MouseEvent) {
                if (selectedBezier == null) return
                val point = Vec2d(e.x, e.y).minus(Vec2d(width / 2, height / 2)).div(ppi)
                val (bezier, type) = selectedBezier!!
                when (type) {
                    BezierPoint.PointType.ANCHOR -> {
                        val diff = point - bezier.anchor
                        bezier.anchor = point
                        bezier.prevHandle = bezier.prevHandle?.plus(diff)
                        bezier.nextHandle = bezier.nextHandle?.plus(diff)
                    }

                    else -> {
                        if (e.isShiftDown) {
                            bezier.mirrored = false
                        }
                        if (e.isAltDown) {
                            bezier.split = true
                        }
                        bezier.updateHandles(type, point)
                    }
                }
                repaint()
            }
        })

        addKeyListener(object : KeyAdapter() {
            override fun keyPressed(e: KeyEvent) {
                when (e.keyCode) {
                    KeyEvent.VK_Z -> {
                        if (e.isControlDown) {
                            if (e.isShiftDown) {
                                redo()
                            } else {
                                undo()
                            }
                        }
                    }
                }
            }
        })
        isFocusable = true
    }

    val points: MutableList<BezierPoint> = mutableListOf(
        BezierPoint(Vec2d(0, 0)),
    )

    private fun updatePoints() {
        // update catmull rom points
        for (i in 0 until points.size - 1) {
            val p1 = points[i]
            if (p1.nextHandle == null && p1.prevHandle != null) {
                // mirror prevHandle
                val diff = p1.prevHandle!! - p1.anchor
                p1.nextHandle = p1.anchor - diff
            }
            val p2 = points[i + 1]

            val p0 = if (i == 0) {
                // if no p0, mirror p2
                val diff = p2.anchor - p1.anchor
                BezierPoint(p1.anchor - diff)
            } else {
                points[i - 1]
            }
            val p3 = if (i == points.size - 2) {
                // if no p3, mirror p1
                val diff = p1.anchor - p2.anchor
                BezierPoint(p2.anchor - diff)
            } else {
                points[i + 2]
            }

            val bezier = Bezier.fromCatmullRom(p0.anchor, p1.anchor, p2.anchor, p3.anchor, 0.5)
            // update the bezier points
            if (!p1.modified) p1.nextHandle = bezier.p1
            if (!p2.modified) p2.prevHandle = bezier.p2
        }
    }

    private val stockState = points.map { it.copy() }

    private val undoStack: MutableList<List<BezierPoint>> = mutableListOf()
    private var stackIndex = -1

    fun addState() {
        updatePoints()
        val slice = undoStack.slice(0..stackIndex)
        undoStack.clear()
        undoStack.addAll(slice)
        undoStack.add(points.map { it.copy() })
        stackIndex = undoStack.size - 1
        println("add ${undoStack.size} $stackIndex")
    }

    fun undo() {
        if (undoStack.isEmpty()) return
        points.clear()
        stackIndex--
        points.addAll((undoStack.getOrNull(stackIndex) ?: stockState).map { it.copy() })
        println("del ${undoStack.size} $stackIndex")
        repaint()
    }

    fun redo() {
        if (undoStack.isEmpty()) return
        points.clear()
        stackIndex++
        points.addAll((undoStack.getOrNull(stackIndex) ?: stockState).map { it.copy() })
        println("res ${undoStack.size} $stackIndex")
//        println(undoStack.joinToString("\n\n") { it.joinToString("\n") })
        repaint()
    }

    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val bufferedImage = BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)
        val g2d = bufferedImage.createGraphics()
        g2d.background = background
        g2d.setRenderingHints(
            mapOf(
                RenderingHints.KEY_ANTIALIASING to RenderingHints.VALUE_ANTIALIAS_ON,
                RenderingHints.KEY_RENDERING to RenderingHints.VALUE_RENDER_QUALITY
            )
        )
        g2d.translate(width / 2, height / 2)

        val red = Color(0xfb1155)
        val blue = Color(0x35b8fa)
        val green = Color(0x11e59c)
        val yellow = Color(0xf1d454)

        val colors = listOf(red, blue, green, yellow)

        var currentColor = 0
        points.windowed(2, 1, false).forEach { (p1, p2) ->
            val bezier = Bezier(p1.anchor, p1.nextHandle!!, p2.prevHandle!!, p2.anchor)
            val color = colors[currentColor]
            g2d.color = color
            currentColor = (currentColor + 1) % colors.size
            val samples = 100
            var lastPoint = bezier.at(0.0) * ppi
            for (i in 0..samples) {
                val t = i.toDouble() / samples
                val point = bezier.at(t) * ppi
                g2d.drawLine(lastPoint.x.toInt(), lastPoint.y.toInt(), point.x.toInt(), point.y.toInt())
                lastPoint = point
            }
        }

        currentColor = 0
        for (point in points) {
            val color1 = colors[(currentColor - 1 + colors.size) % colors.size]
            val color2 = colors[currentColor]
            currentColor = (currentColor + 1) % colors.size
            point.draw(g2d, ppi, color1, color2)
        }

        g.drawImage(bufferedImage, 0, 0, null)
    }
}