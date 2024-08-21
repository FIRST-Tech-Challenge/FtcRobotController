package com.millburnx.pathplanner

import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Utils
import java.awt.Color
import java.awt.FlowLayout
import java.awt.Font
import java.awt.Rectangle
import javax.swing.JButton
import javax.swing.JPanel

class ButtonPanelWrapper(val pathPlanner: PathPlanner) : JPanel() {
    class ButtonPanel(
        val scale: Double,
        val getPoints: () -> List<BezierPoint>,
        val setPoints: (List<BezierPoint>) -> Unit
    ) :
        JPanel() {
        init {
            layout = FlowLayout(FlowLayout.CENTER)

            val buttons = listOf(
                JButton("Load Path").apply {
                    addActionListener {
                        val file = Utils.fileDialog("paths", "*.tsv", false) ?: return@addActionListener
                        val newPath = BezierPoint.loadFromTSV(file)
                        setPoints(newPath)
                    }
                },
                JButton("Save Path").apply {
                    addActionListener {
                        val file = Utils.fileDialog("paths", "*.tsv", true) ?: return@addActionListener
                        BezierPoint.saveToTSV(file, getPoints())
                    }
                }
            )

            buttons.forEach {
                it.font = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt())
                add(it)
            }
        }
    }

    val buttonPanel: ButtonPanel = ButtonPanel(pathPlanner.scale, { pathPlanner.bezierPoints }) {
        pathPlanner.setPoints(it)
    }

    init {
        layout = FlowLayout(FlowLayout.CENTER)
        background = Color(0, 0, 0, 0)

        add(buttonPanel)
        updateSize()
    }

    fun updateSize() {
        val height = preferredSize.height
        val width = pathPlanner.width
        bounds = Rectangle(0, 0, width, height)
        revalidate()
        repaint()
    }
}