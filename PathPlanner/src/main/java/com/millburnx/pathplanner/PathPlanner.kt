package com.millburnx.pathplanner

import com.millburnx.dashboard.JPopover
import com.millburnx.utils.Bezier
import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.Graphics
import java.awt.RenderingHints
import java.awt.event.ComponentAdapter
import java.awt.event.ComponentEvent
import java.awt.image.BufferedImage
import javax.imageio.ImageIO
import javax.swing.JPanel

class PathPlanner(var ppi: Double, val scale: Double) : JPanel() {
    val drawImage = false
    val backgroundImage = ImageIO.read(javaClass.classLoader.getResource("bg.png"))
    val bezierPoints: MutableList<BezierPoint> = mutableListOf()
    val historyStack: MutableList<List<BezierPoint>> = mutableListOf()
    var currentPopoverRef: JPopover? = null
    private val listeners: PathPlannerListeners = PathPlannerListeners(this)
    val buttonPanel: ButtonPanelWrapper = ButtonPanelWrapper(this)

    init {
        addMouseListener(listeners.mouse)
        addMouseMotionListener(listeners.mouseMotion)
        addKeyListener(listeners.key)

        layout = null
        add(buttonPanel)

        addComponentListener(object : ComponentAdapter() {
            override fun componentResized(e: ComponentEvent?) {
                buttonPanel.updateSize()
                repaint()
            }
        })
    }

    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val bufferedImage = BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)
        val g2d = bufferedImage.createGraphics()
        g2d.setRenderingHints(
            RenderingHints(
                RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON
            )
        )
        g2d.addRenderingHints(
            RenderingHints(
                RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON
            )
        )

        g2d.background = Utils.Colors.bg1
        g2d.clearRect(0, 0, width, height)
        if (drawImage) {
            g2d.drawImage(backgroundImage, 0, 0, width, height, null)
        } else {
            val grid = Vec2d(6, 6)
            g2d.color = Utils.Colors.bg2
            for (i in 0 until width step (width / grid.x).toInt()) {
                g2d.drawLine(i, 0, i, height)
            }
            for (i in 0 until height step (height / grid.y).toInt()) {
                g2d.drawLine(0, i, width, i)
            }
        }
        g2d.translate(width / 2, height / 2)

        bezierPoints.zipWithNext() { p1, p2 ->
            Bezier(p1.anchor, p1.nextHandle!!, p2.prevHandle!!, p2.anchor)
        }.forEach {
            it.g2dDraw(g2d, ppi, scale, Utils.Colors.green)
        }

        for (bezierPoint in bezierPoints) {
            bezierPoint.draw(g2d, ppi, scale, Utils.Colors.red, Utils.Colors.blue)
        }

        g.drawImage(bufferedImage, 0, 0, null)
    }

    fun updateCatmullRom() {
        for (i in 0 until bezierPoints.size - 1) {
            val p1 = bezierPoints[i]
            val p2 = bezierPoints[i + 1]

            val p0 = bezierPoints.getOrNull(i - 1) ?: BezierPoint(p1.anchor - (p2.anchor - p1.anchor))
            val p3 = bezierPoints.getOrNull(i + 2) ?: BezierPoint(p2.anchor - (p1.anchor - p2.anchor))

            val bezier = Bezier.fromCatmullRom(p0.anchor, p1.anchor, p2.anchor, p3.anchor)
            if (!p1.modified) {
                p1.nextHandle = bezier.p1
            } else if (p1.nextHandle == null) {
                // mirror
                p1.nextHandle = p1.anchor + (p1.anchor - p1.prevHandle!!)
            }
            if (!p2.modified) {
                p2.prevHandle = bezier.p2
            } else if (p2.prevHandle == null) {
                // mirror
                p2.prevHandle = p2.anchor + (p2.anchor - p2.nextHandle!!)
            }
        }
    }

    fun setPoints(points: List<BezierPoint>) {
        bezierPoints.clear()
        bezierPoints.addAll(points)
        updateCatmullRom()
        repaint()
    }

    fun addPoint(bezierPoint: BezierPoint) {
        bezierPoints.add(bezierPoint)
        updateCatmullRom()
        repaint()
    }

    fun removePoint(bezierPoint: BezierPoint) {
        bezierPoints.remove(bezierPoint)
        updateCatmullRom()
        repaint()
    }

    fun addPopover(position: Vec2d, target: Pair<BezierPoint, BezierPoint.PointType>) {
        currentPopoverRef = JPopover(this, position, target.first, target.second, ppi, scale) {
            target.first.updateType(target.second, it)
            updateCatmullRom()
            
            repaint()
        }
        add(currentPopoverRef)
        repaint()
    }
}