package com.millburnx.dashboard

import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Dimension
import java.awt.Font
import java.awt.Graphics
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.Insets
import java.awt.Rectangle
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import javax.swing.JCheckBox
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JSpinner
import javax.swing.SpinnerNumberModel
import javax.swing.border.EmptyBorder
import javax.swing.text.NumberFormatter

class JNumber(
    val value: Double,
    min: Double,
    max: Double,
    step: Double,
    val borderColor: Color = Utils.Colors.bg2,
    val borderRadius: Double = 6.0,
    val borderWidth: Double = 1.0,
    val padding: Vec2d = Vec2d(12, 8),
    val scale: Double = 1.0,
    val editorFont: Font = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt()),
    val editorBackground: Color = Utils.Colors.bg0,
    val spacing: Double = 8.0
) {
    val model = SpinnerNumberModel(value, min, max, step)
    val spinner = object : JSpinner(model) {
        init {
            border = EmptyBorder(padding.y.toInt(), padding.x.toInt(), padding.y.toInt(), padding.x.toInt())
            font = editorFont
        }

        override fun paintComponent(g: Graphics) {
            val (bufferedImage, g2d) = Utils.bufferedImage(width, height)

            g2d.stroke = BasicStroke(borderWidth.toFloat())
            g2d.color = editorBackground
            g2d.fillRoundRect(0, 0, width, height, (borderRadius * scale).toInt(), (borderRadius * scale).toInt())
            g2d.color = borderColor
            g2d.drawRoundRect(
                ((borderWidth * scale / 2).toInt()),
                ((borderWidth * scale / 2).toInt()),
                (width - borderWidth).toInt(),
                (height - borderWidth).toInt(),
                (borderRadius * scale).toInt(),
                (borderRadius * scale).toInt()
            )

            super.paintComponent(g2d)

            g.drawImage(bufferedImage, 0, 0, null)
        }
    }

    init {
        spinner.editor.background = editorBackground
        spinner.editor.border = EmptyBorder(0, 0, 0, spacing.toInt())
        val textField = (spinner.editor as JSpinner.NumberEditor).textField
        textField.columns = 3
        val formatter = (textField.formatter as NumberFormatter)
        formatter.allowsInvalid = false
        formatter.commitsOnValidEdit = true
    }
}

class JPopover(
    val parentPanel: JPanel,
    pos: Vec2d,
    point: BezierPoint,
    type: BezierPoint.PointType,
    val ppi: Double,
    val scale: Double,
    updateCallback: (
        value: Vec2d
    ) -> Unit
) : JPanel() {
    val popoverBackground = Utils.Colors.bg0
    val borderWidth = 1.0
    val borderRadius = 6.0
    val borderColor = Utils.Colors.bg2
    val padding = Vec2d(16)
    val textFont = Font("Noto Sans", Font.PLAIN, (16 * scale).toInt())
    val margin = 8

    init {
        // list all fonts
        location = pos.awt()
        background = Color(0, 0, 0, 0) // Transparent to allow custom background drawing

        val outer = padding + Vec2d(borderWidth)
        val outerX = outer.x.toInt()
        val outerY = outer.y.toInt()
        border = EmptyBorder(outerY, outerX, outerY, outerX)

        layout = GridBagLayout()
        val constraints = GridBagConstraints()
        val gaps = Vec2d(8)
        constraints.insets = Insets(gaps.y.toInt(), gaps.x.toInt(), gaps.y.toInt(), gaps.x.toInt())
        constraints.anchor = GridBagConstraints.WEST
        // 2 columns

        var currentPoint = point.getType(type)!!

        font = textFont
        val xLabel = JLabel("X")
        xLabel.isDoubleBuffered = true
        xLabel.font = textFont
        add(xLabel, constraints)

        val xNumber = JNumber(currentPoint.x, -144 / 2.0, 144 / 2.0, 1.0, scale = scale)
        val xInput = xNumber.spinner
        xInput.font = textFont
        xInput.addChangeListener() {
            val newValue = xNumber.model.value as Double
            updateCallback(Vec2d(newValue, currentPoint.y))
            val parentSize = Vec2d(parentPanel.width, parentPanel.height)
            setPosition(Vec2d(newValue, currentPoint.y) * ppi + parentSize / 2)
        }

        constraints.gridx = 1
        add(xInput, constraints)

        val yLabel = JLabel("Y")
        yLabel.isDoubleBuffered = true
        yLabel.font = textFont
        constraints.gridx = 0
        constraints.gridy = 1
        add(yLabel, constraints)

        val yNumber = JNumber(currentPoint.y, -144 / 2.0, 144 / 2.0, 1.0, scale = scale)
        val yInput = yNumber.spinner
        yInput.font = textFont
        yInput.addChangeListener() {
            val newValue = yNumber.model.value as Double
            updateCallback(Vec2d(currentPoint.x, newValue))
            val parentSize = Vec2d(parentPanel.width, parentPanel.height)
            setPosition(Vec2d(currentPoint.x, newValue) * ppi + parentSize / 2)
        }
        constraints.gridx = 1
        add(yInput, constraints)

        val mirroredSwitch = JCheckBox()
        mirroredSwitch.isSelected = point.mirrored
        mirroredSwitch.size = Dimension((16 * scale).toInt(), (16 * scale).toInt())
        mirroredSwitch.addChangeListener {
            point.mirrored = mirroredSwitch.isSelected
            updateCallback(currentPoint)
        }
        constraints.gridy++
        add(mirroredSwitch, constraints)

        val mirroredLabel = JLabel("Mirrored")
        mirroredLabel.isDoubleBuffered = true
        mirroredLabel.font = textFont
        mirroredLabel.addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) {
                point.mirrored = !point.mirrored
                mirroredSwitch.isSelected = point.mirrored
                updateCallback(currentPoint)
            }
        })
        constraints.gridx = 0
        add(mirroredLabel, constraints)

        val splitSwitch = JCheckBox()
        splitSwitch.isSelected = point.split
        splitSwitch.size = Dimension((16 * scale).toInt(), (16 * scale).toInt())
        splitSwitch.addChangeListener {
            point.split = splitSwitch.isSelected
            updateCallback(currentPoint)
        }
        constraints.gridy++
        add(splitSwitch, constraints)

        val splitLabel = JLabel("Split")
        splitLabel.isDoubleBuffered = true
        splitLabel.font = textFont
        splitLabel.addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) {
                point.split = !point.split
                splitSwitch.isSelected = point.split
                updateCallback(currentPoint)
            }
        })

        constraints.gridx = 0
        add(splitLabel, constraints)

        setPosition(pos)

        addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) {
                println("Popover clicked") // block bubbling
            }
        })
    }

    fun closePopover() {
        parentPanel.remove(this)
        parentPanel.revalidate()
        parentPanel.repaint()
    }

    fun setPosition(origin: Vec2d) {
        var optimalPos = origin.copy() + margin
        val preferred = preferredSize
        val size = Vec2d(preferred.width, preferred.height) + margin * 2
        if (origin.y + size.y > parentPanel.height) {
            val topSpace = origin.y - size.y
            if (topSpace > 0) {
                optimalPos = Vec2d(origin.x - size.x, topSpace)
            } else {
                optimalPos = Vec2d(origin.x - size.x, parentPanel.height - size.y)
            }
        }
        if (origin.x + size.x > parentPanel.width) {
            val leftSpace = origin.x - size.x
            if (leftSpace > 0) {
                optimalPos = Vec2d(leftSpace, optimalPos.y)
            } else {
                optimalPos = Vec2d(parentPanel.width - size.x, optimalPos.y)
            }
        }

        bounds = Rectangle(optimalPos.awt(), Dimension(size.x.toInt(), size.y.toInt()))
        revalidate()
        repaint()
    }

    override fun paintComponent(g: Graphics) {
        val (bufferedImage, g2d) = Utils.bufferedImage(width, height)

        g2d.stroke = BasicStroke(borderWidth.toFloat())
        g2d.color = popoverBackground
        g2d.fillRoundRect(0, 0, width, height, 10, 10)
        g2d.color = borderColor
        g2d.drawRoundRect(
            (borderWidth * scale / 2).toInt(),
            (borderWidth * scale / 2).toInt(),
            (width - borderWidth * scale).toInt(),
            (height - borderWidth * scale).toInt(),
            (borderRadius * scale).toInt(),
            (borderRadius * scale).toInt()
        )

        super.paintComponent(g2d)

        g.drawImage(bufferedImage, 0, 0, null)
    }
}