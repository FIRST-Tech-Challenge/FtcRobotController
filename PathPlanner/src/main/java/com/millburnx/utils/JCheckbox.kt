package com.millburnx.utils

import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics
import javax.swing.JCheckBox
import javax.swing.border.EmptyBorder

/**
 * A custom styled JCheckBox
 */
class JCheckbox(
    val scale: Double,
    val checked: Boolean = false,
    val bgColor: Color = Color.decode(Utils.Colors.bg0),
    val filledColor: Color = Color.decode(Utils.Colors.blue),
    val borderColor: Color = Color.decode(Utils.Colors.bg2),
    val borderRadius: Double = 6.0,
    val borderWidth: Float = 1f,
    val padding: Vec2d = Vec2d(2),
    val icon: List<Vec2d> = listOf(
        Vec2d(4.5, 12.75),
        Vec2d(10.5, 18.75),
        Vec2d(19.5, 5.25),
    ).map { it / 24 }, // from HeroIcons
) : JCheckBox() {
    init {
        border = EmptyBorder(((padding + borderWidth) * scale).insets())
        isSelected = checked
    }

    override fun paintComponent(g: Graphics) {
        val (bufferedImage, g2d) = Utils.bufferedImage(width, height)
        val regularFill = if (isSelected) filledColor else bgColor
        val disabledFill = Color.decode(if (isSelected) Utils.Colors.bg4 else Utils.Colors.bg2)
        val fillColor = if (isEnabled) regularFill else disabledFill
        Utils.drawRoundedPanel(
            g2d, scale, Vec2d(width, height), fillColor, borderRadius, borderColor, borderWidth
        )
        if (isSelected) {
            g2d.color = Color.WHITE
            g2d.stroke = BasicStroke(2f)
            val xPoints = icon.map { (it.x * width).toInt() }.toIntArray()
            val yPoints = icon.map { (it.y * height).toInt() }.toIntArray()
            g2d.drawPolyline(
                xPoints,
                yPoints,
                icon.size
            )
        }
        g.drawImage(bufferedImage, 0, 0, null)
    }
}