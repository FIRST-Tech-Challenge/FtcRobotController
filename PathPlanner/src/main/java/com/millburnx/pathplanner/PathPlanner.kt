package com.millburnx.pathplanner

import com.millburnx.dashboard.JPopover
import com.millburnx.utils.Bezier
import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.BasicStroke
import java.awt.Color
import java.awt.FileDialog
import java.awt.FlowLayout
import java.awt.Font
import java.awt.Graphics
import java.awt.Rectangle
import java.awt.RenderingHints
import java.awt.event.KeyAdapter
import java.awt.event.KeyEvent
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.image.BufferedImage
import java.io.File
import javax.swing.JButton
import javax.swing.JPanel

class PathPlanner(var ppi: Double, val scale: Double) : JPanel() {
    var currentEditPopover: JPopover? = null
    val buttonPanel: JPanel

    init {
        background = Utils.Colors.bg1

        // the bezier point and the actual point (anchor, prevHandle, nextHandle)
        var selectedBezier: Pair<BezierPoint, BezierPoint.PointType>? = null

        val currentPanel = this
        addMouseListener(object : MouseAdapter() {
            override fun mousePressed(e: MouseEvent) {
                println("mouse pressed")
                val clickPoint = Vec2d(e.x, e.y).minus(Vec2d(width / 2, height / 2)).div(ppi)
                var popupRemoved = false
                if (currentEditPopover != null) {
                    remove(currentEditPopover)
                    currentEditPopover = null
                    revalidate()
                    repaint()
                    popupRemoved = true
                }
                val anchorExtraThreshold = 6
                val handleExtraThreshold = 3
                val anchorSize = 2.5
                val handleSize = 2.5
                when (e.button) {
                    MouseEvent.BUTTON1 -> {
                        if (popupRemoved) return
                        for (point in points) {
                            for (type in BezierPoint.PointType.entries) {
                                val threshold = when (type) {
                                    BezierPoint.PointType.ANCHOR -> anchorSize / 2 + anchorExtraThreshold / ppi * scale
                                    else -> handleSize / 2 + handleExtraThreshold / ppi * scale
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

                    MouseEvent.BUTTON2 -> {
                        for (point in points) {
                            for (type in BezierPoint.PointType.entries) {
                                val threshold = when (type) {
                                    BezierPoint.PointType.ANCHOR -> anchorSize / 2 + anchorExtraThreshold / ppi * scale
                                    else -> handleSize / 2 + handleExtraThreshold / ppi * scale
                                }
                                val distance = point.getType(type)?.distanceTo(clickPoint) ?: Double.POSITIVE_INFINITY
                                if (distance < threshold) {
                                    val editPopover = JPopover(currentPanel, Vec2d(e.x, e.y), point, type, ppi, scale) {
                                        println("update $it")
                                        when (type) {
                                            BezierPoint.PointType.ANCHOR -> {
                                                point.anchor = it
                                                point.prevHandle?.plus(it - point.anchor)?.let { point.prevHandle = it }
                                                point.nextHandle?.plus(it - point.anchor)?.let { point.nextHandle = it }
                                                if (point.prevHandle != null) {
                                                    point.updateHandles(
                                                        BezierPoint.PointType.PREV_HANDLE,
                                                        point.prevHandle!!
                                                    )
                                                } else {
                                                    point.updateHandles(
                                                        BezierPoint.PointType.NEXT_HANDLE,
                                                        point.nextHandle!!
                                                    )
                                                }
                                            }

                                            else -> point.updateHandles(type, it)
                                        }
                                        addState()
                                        repaint()
                                    }
                                    add(editPopover, null)
                                    revalidate()
                                    repaint()
                                    currentEditPopover = editPopover
                                    return
                                }
                            }
                        }
                    }

                    MouseEvent.BUTTON3 -> {
                        if (popupRemoved) return
                        for (point in points) {
                            val threshold = anchorSize / 2 + anchorExtraThreshold / ppi * scale
                            val distance = point.anchor.distanceTo(clickPoint)
                            if (distance < threshold) {
                                points.remove(point)
                                // remove stray handles
                                if (points.isNotEmpty()) {
                                    points.first().prevHandle = null
                                    points.last().nextHandle = null
                                }
                                addState()
                                repaint()
                                return
                            }
                        }
                    }
                }
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

        layout = null;

        buttonPanel = JPanel()
        buttonPanel.background = Color(0, 0, 0, 0)
        buttonPanel.layout = FlowLayout(FlowLayout.CENTER)

        val innerButtonPanel = JPanel()
        innerButtonPanel.layout = FlowLayout(FlowLayout.CENTER)
        val loadButton = JButton("Load")
        loadButton.font = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt())
        loadButton.addActionListener { loadTSV() }
        innerButtonPanel.add(loadButton)

        val saveButton = JButton("Save")
        saveButton.font = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt())

        saveButton.addActionListener { saveTSV() }
        innerButtonPanel.add(saveButton)
        buttonPanel.add(innerButtonPanel)

        updateButtonBounds()
        add(buttonPanel)
    }

    fun updateButtonBounds() {
        val preferredHeight = buttonPanel.preferredSize.height
        val bounds = Rectangle(0, 0, width, preferredHeight)
        buttonPanel.setBounds(bounds)
        revalidate()
        repaint()
    }

    fun loadTSV() {
        val fileDialog = FileDialog(null as java.awt.Frame?, "Select a file", FileDialog.LOAD)
        fileDialog.directory = File("paths").absolutePath
        fileDialog.file = "*.tsv"
        fileDialog.isVisible = true
        val file = fileDialog.file
        if (file == null) {
            println("No file selected")
            return
        }
        val pathFile = File(fileDialog.directory, file)
        val pathPoints = Vec2d.loadList(pathFile)
        val newPath: MutableList<BezierPoint> = mutableListOf()
        for (i in 0..<pathPoints.size step 3) {
            val anchor = pathPoints[i]
            val prevHandle = pathPoints.getOrNull(i - 1)
            val nextHandle = pathPoints.getOrNull(i + 1)
            newPath.add(BezierPoint(anchor, prevHandle, nextHandle, true))
        }
        points.clear()
        points.addAll(newPath)
        addState()
        repaint()
    }

    fun saveTSV() {
        val fileDialog = FileDialog(null as java.awt.Frame?, "Select a file", FileDialog.SAVE)
        fileDialog.directory = File("paths").absolutePath
        fileDialog.file = "*.tsv"
        fileDialog.isVisible = true
        val file = fileDialog.file
        if (file == null) {
            println("No file selected")
            return
        }
        val pathFile = File(fileDialog.directory, file)
        val points = points.map { listOf(it.prevHandle, it.anchor, it.nextHandle) }.flatten().filterNotNull()
        Vec2d.saveList(points, pathFile)
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
        // draw grid
        g2d.color = Utils.Colors.bg2
        val size = Vec2d(6, 6)
        for (x in 0..width step width / size.x.toInt()) {
            g2d.drawLine(x - width / 2, -height / 2, x - width / 2, height / 2)
        }
        for (y in 0..height step height / size.y.toInt()) {
            g2d.drawLine(-width / 2, y - height / 2, width / 2, y - height / 2)
        }

        val colors = listOf(
            Utils.Colors.red,
            Utils.Colors.blue,
            Utils.Colors.green,
            Utils.Colors.yellow
        )

        var currentColor = 0
        g2d.stroke = BasicStroke((0.1 * scale * ppi).toFloat())
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
            point.draw(g2d, ppi, scale, color1, color2)
        }

        g.drawImage(bufferedImage, 0, 0, null)
    }
}