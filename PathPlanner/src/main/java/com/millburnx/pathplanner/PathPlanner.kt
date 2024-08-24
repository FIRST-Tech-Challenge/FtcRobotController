package com.millburnx.pathplanner

import com.millburnx.utils.Bezier
import com.millburnx.utils.BezierPoint
import com.millburnx.utils.GridBagHelper
import com.millburnx.utils.JCheckbox
import com.millburnx.utils.JNumber
import com.millburnx.utils.JPopover
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.Color
import java.awt.Font
import java.awt.Graphics
import java.awt.GridBagLayout
import java.awt.event.ComponentAdapter
import java.awt.event.ComponentEvent
import javax.imageio.ImageIO
import javax.swing.JPanel
import kotlin.math.round

class PathPlanner(var ppi: Double, val scale: Double) : JPanel() {
    val drawImage = false
    val backgroundImage = ImageIO.read(javaClass.classLoader.getResource("bg.png"))
    val bezierPoints: MutableList<BezierPoint> = mutableListOf()
    val undoStack: MutableList<List<Change>> = mutableListOf()
    val redoStack: MutableList<List<Change>> = mutableListOf()
    var currentPopoverRef: JPopover? = null
    private val listeners: PathPlannerListeners = PathPlannerListeners(this)
    val buttonPanel: ButtonPanelWrapper = ButtonPanelWrapper(this)

    init {
        addMouseListener(listeners.mouse)
        addMouseMotionListener(listeners.mouseMotion)
        addKeyListener(listeners.key)

        isFocusable = true
        layout = null
        font = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt())
        foreground = Color.white
        add(buttonPanel)

        addComponentListener(object : ComponentAdapter() {
            override fun componentResized(e: ComponentEvent?) {
                buttonPanel.updateSize()
                repaint()
            }
        })
    }

    override fun paintComponent(g: Graphics) {
        val (bufferedImage, g2d) = Utils.bufferedImage(width, height)
        super.paintComponent(g2d)

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
//        bezierPoints.add(bezierPoint)
        val change = PointAddition(this, bezierPoint, bezierPoints.size)
        addChanges(listOf(change))
        change.apply()
        updateCatmullRom()
        repaint()
    }

    fun removePoint(bezierPoint: BezierPoint) {
        val index = bezierPoints.indexOf(bezierPoint)
        val change = PointRemoval(this, bezierPoint, index)
        addChanges(listOf(change))
        change.apply()
        updateCatmullRom()
        repaint()
    }

    fun _removePoint(bezierPoint: BezierPoint) {
        val index = bezierPoints.indexOf(bezierPoint)
        bezierPoints.remove(bezierPoint)
        if (bezierPoints.isEmpty()) return
        val wasFirst = index == 0
        val wasLast = index == bezierPoints.size
        if (wasFirst) {
            bezierPoints.first().prevHandle = null
        }
        if (wasLast) {
            bezierPoints.last().nextHandle = null
        }
    }

    /**
     * Add a change to the undo stack and clears the redo stack. Does NOT apply the change (mainly because of the point translation/modifications)
     */
    fun addChanges(changes: List<Change>) {
        undoStack.add(changes)
        redoStack.clear()
    }

    fun undo() {
        if (undoStack.isEmpty()) return
        val change = undoStack.removeLast()
        println("Undo: $change")
        change.forEach { it.undo() }
        redoStack.add(change)
        updateCatmullRom()
        repaint()
    }

    fun redo() {
        if (redoStack.isEmpty()) return
        val change = redoStack.removeLast()
        println("Redo: $change")
        change.forEach { it.apply() }
        undoStack.add(change)
        updateCatmullRom()
        repaint()
    }

    fun addPopover(position: Vec2d, target: Pair<BezierPoint, BezierPoint.PointType>) {
        val bezierPoint = target.first
        val type = target.second
        val point = bezierPoint.getType(type)!!

        val popover = JPopover(this, scale)
        val wrapper = JPanel()
        wrapper.background = Utils.Colors.bg0
        wrapper.font = font
        popover.add(wrapper)

        wrapper.layout = GridBagLayout()
        val gridBag = GridBagHelper(wrapper, 2)
        gridBag.constraints.insets = Vec2d(8).insets()

        val xLabel = Utils.jLabel("X", font, foreground)
        val xNumber = JNumber(wrapper, scale, -144.0 / 2, 144.0 / 2, 1.0, point.x)
        xNumber.spinner.addChangeListener() {
            val value = xNumber.model.number.toDouble()
            val diff = value - bezierPoint.getType(type)!!.x
            if (round(diff * 100) / 100 == 0.0) return@addChangeListener
            println("X: ${xNumber.model.number.toDouble()}")
            val translation = PointTranslation(bezierPoint, type, Vec2d(diff, 0.0))
            addChanges(listOf(translation))
            translation.apply()
            updateCatmullRom()
            repaint()
        }

        val yLabel = Utils.jLabel("Y", font, foreground)
        val yNumber = JNumber(wrapper, scale, -144.0 / 2, 144.0 / 2, 1.0, point.y)
        yNumber.spinner.addChangeListener() {
            val value = yNumber.model.number.toDouble()
            val diff = value - bezierPoint.getType(type)!!.y
            if (round(diff * 100) / 100 == 0.0) return@addChangeListener
            println("Y: ${yNumber.model.number.toDouble()}")
            val translation = PointTranslation(bezierPoint, type, Vec2d(0.0, diff))
            addChanges(listOf(translation))
            translation.apply()
            updateCatmullRom()
            repaint()
        }

        gridBag.addAll(listOf(xLabel, xNumber.spinner, yLabel, yNumber.spinner))

        val splitLabel = Utils.jLabel("Split", font, foreground)
        val split = JCheckbox(scale, checked = bezierPoint.split)
        split.addActionListener {
            if (split.isSelected == bezierPoint.split) return@addActionListener
            println("Split: ${split.isSelected}")
            val modification = PointModification(bezierPoint, split = split.isSelected)
            addChanges(listOf(modification))
            modification.apply()
            updateCatmullRom()
            repaint()
        }
        split.isEnabled = !bezierPoint.mirrored

        val mirrorLabel = Utils.jLabel("Mirror", font, foreground)
        val mirror = JCheckbox(scale, checked = bezierPoint.mirrored)
        mirror.addActionListener {
            if (mirror.isSelected == bezierPoint.mirrored) return@addActionListener
            println("Mirror: ${mirror.isSelected}")
            split.isEnabled = !mirror.isSelected
            val modification = PointModification(bezierPoint, mirrored = mirror.isSelected)
            addChanges(listOf(modification))
            modification.apply()
            updateCatmullRom()
            repaint()
        }

        gridBag.addAll(listOf(mirrorLabel, mirror, splitLabel, split))

        popover.add()
        popover.show(position)
        currentPopoverRef = popover
    }
}