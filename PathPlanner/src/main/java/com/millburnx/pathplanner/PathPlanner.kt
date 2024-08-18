package com.millburnx.pathplanner

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
                if (e.button != MouseEvent.BUTTON1) return;

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
                            return;
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
            fun updateHandle(bezier: BezierPoint, type: BezierPoint.PointType, newPoint: Vec2d) {
                if (type == BezierPoint.PointType.ANCHOR) {
                    // it's called updateHandle and not updatePoint for a reason
                    throw IllegalArgumentException("Do not call updateHandle with an anchor point")
                }
                bezier.modified = true
                if (!bezier.split && !bezier.mirrored) {
                    val oppositeType = type.opposite()
                    val oppositePoint = bezier.getType(oppositeType)
                    if (oppositePoint != null) {
                        val distance = oppositePoint.distanceTo(bezier.anchor)
                        val angle = newPoint.angleTo(bezier.anchor)
                        val newOppositePoint = Vec2d(distance, 0.0).rotate(angle) + bezier.anchor
                        bezier.setType(oppositeType, newOppositePoint)
                    }
                }
                bezier.setType(type, newPoint)
                if (bezier.split || !bezier.mirrored) {
                    return;
                }
                val oppositeType = type.opposite()
                val newDiff = newPoint - bezier.anchor
                bezier.setType(oppositeType, bezier.anchor - newDiff)
                return;
            }

            override fun mouseDragged(e: MouseEvent) {
                if (selectedBezier == null) return;
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
                        updateHandle(bezier, type, point)
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
                            undo()
                        }
                    }
                }
            }
        })
        isFocusable = true
    }

    val points: MutableList<BezierPoint> = mutableListOf(
        BezierPoint(Vec2d(0, 0)),
        BezierPoint(Vec2d(0, -12), Vec2d(-12, -12), Vec2d(12, -12)),
    )
    val stockState = points.map { it.copy() }

    val undoStack: ArrayDeque<List<BezierPoint>> = ArrayDeque()

    fun addState() {
        undoStack.add(points.map { it.copy() })
        println("add ${undoStack.size}")
    }

    fun undo() {
        if (undoStack.isEmpty()) return;
        points.clear()
        undoStack.removeLast()
        points.addAll(undoStack.lastOrNull() ?: stockState.map { it.copy() })
        println("del ${undoStack.size}")
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

        for (point in points) {
            point.draw(g2d, ppi, red, blue)
        }

        g.drawImage(bufferedImage, 0, 0, null)
    }
}