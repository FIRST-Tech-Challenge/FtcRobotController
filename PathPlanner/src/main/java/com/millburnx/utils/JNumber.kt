package com.millburnx.utils

import java.awt.Color
import java.awt.Graphics
import javax.swing.JPanel
import javax.swing.JSpinner
import javax.swing.SpinnerNumberModel
import javax.swing.border.EmptyBorder
import javax.swing.text.NumberFormatter

class JNumber(
    val parentPanel: JPanel, val scale: Double,
    min: Double, max: Double, step: Double, value: Double,
    val bgColor: Color = Utils.Colors.bg0, val fgColor: Color = Color.WHITE,
    val borderColor: Color = Utils.Colors.bg2, val borderRadius: Double = 6.0, val borderWidth: Float = 1f,
    val padding: Vec2d = Vec2d(8), val spacing: Double = 8.0
) {
    val model: SpinnerNumberModel = SpinnerNumberModel(value, min, max, step)
    val spinner = object : JSpinner(model) {
        init {
            background = Color(0, 0, 0, 0)
            font = parentPanel.font
            foreground = fgColor

            border = EmptyBorder(((padding + borderWidth) * scale).insets())

            editor.background = bgColor
            editor.foreground = fgColor
            editor.border = EmptyBorder(0, 0, 0, (spacing * scale).toInt())

            val textField = (editor as NumberEditor).textField
            textField.caret.isSelectionVisible = true
            textField.caretColor = fgColor // IDK why this is off by default at least for gtk, otherwise it's invisible

            textField.columns = 4
            val formatter = textField.formatter as NumberFormatter
            formatter.allowsInvalid = false
            formatter.commitsOnValidEdit = true
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
}