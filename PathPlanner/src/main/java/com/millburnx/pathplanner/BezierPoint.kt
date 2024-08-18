package com.millburnx.pathplanner

import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics2D

class BezierPoint(
    var anchor: Vec2d,
    var prevHandle: Vec2d? = null,
    var nextHandle: Vec2d? = null,
    var modified: Boolean = false,
    var mirrored: Boolean = true,
    var split: Boolean = false
) {
    enum class PointType {
        ANCHOR,
        PREV_HANDLE,
        NEXT_HANDLE
    }

    fun getType(pointType: PointType): Vec2d? {
        return when (pointType) {
            PointType.ANCHOR -> anchor
            PointType.PREV_HANDLE -> prevHandle
            PointType.NEXT_HANDLE -> nextHandle
        }
    }

    fun setType(pointType: PointType, value: Vec2d?) {
        when (pointType) {
            PointType.ANCHOR -> anchor = value!!
            PointType.PREV_HANDLE -> prevHandle = value
            PointType.NEXT_HANDLE -> nextHandle = value
        }
    }

    fun draw(g2d: Graphics2D, ppi: Double, prevColor: Color, nextColor: Color) {
        // draw the previous handle
        for (type in listOf(PointType.PREV_HANDLE, PointType.NEXT_HANDLE)) {
            val handle = getType(type) ?: continue
            val color = when (type) {
                PointType.PREV_HANDLE -> prevColor
                PointType.NEXT_HANDLE -> nextColor
                else -> continue
            }
            g2d.color = color
            Utils.drawLine(g2d, ppi, anchor, handle)
            g2d.color = g2d.background
            Utils.drawPoint(g2d, ppi, handle, 2.5)
            g2d.color = color
            Utils.drawPoint(g2d, ppi, handle, 2.5, false)
        }

        // draw anchor point
        g2d.color = g2d.background
        Utils.drawPoint(g2d, ppi, anchor, 2.5)
        g2d.color = Color.WHITE
        g2d.stroke = BasicStroke(2.0f)
        Utils.drawPoint(g2d, ppi, anchor, 2.5, false)
    }
}