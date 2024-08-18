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
        if (prevHandle != null) {
            g2d.color = prevColor
            Utils.drawLine(g2d, ppi, anchor, prevHandle!!)
            g2d.color = g2d.background
            Utils.drawPoint(g2d, ppi, prevHandle!!, 2.5)
            g2d.color = prevColor
            Utils.drawPoint(g2d, ppi, prevHandle!!, 2.5, false)
        }

        // draw the next handle
        if (nextHandle != null) {
            g2d.color = nextColor
            Utils.drawLine(g2d, ppi, anchor, nextHandle!!)
            g2d.color = g2d.background
            Utils.drawPoint(g2d, ppi, nextHandle!!, 2.5)
            g2d.color = nextColor
            Utils.drawPoint(g2d, ppi, nextHandle!!, 2.5, false)
        }

        // draw anchor point
        g2d.color = g2d.background
        Utils.drawPoint(g2d, ppi, anchor, 2.5)
        g2d.color = Color.WHITE
        g2d.stroke = BasicStroke(2.0f)
        Utils.drawPoint(g2d, ppi, anchor, 2.5, false)
    }
}