package com.millburnx.utils

import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics2D
import java.io.File

class BezierPoint(
    var anchor: Vec2d,
    var prevHandle: Vec2d? = null,
    var nextHandle: Vec2d? = null,
    var modified: Boolean = false,
    var mirrored: Boolean = true,
    var split: Boolean = false
) {
    companion object {
        fun saveToTSV(file: File, points: List<BezierPoint>) {
            val points = points.map { listOf(it.prevHandle, it.anchor, it.nextHandle) }.flatten().filterNotNull()
            Vec2d.saveList(points, file)
        }

        fun loadFromTSV(file: File): List<BezierPoint> {
            val pathPoints = Vec2d.loadList(file)
            val newPath: MutableList<BezierPoint> = mutableListOf()
            for (i in 0..<pathPoints.size step 3) {
                val anchor = pathPoints[i]
                val prevHandle = pathPoints.getOrNull(i - 1)
                val nextHandle = pathPoints.getOrNull(i + 1)
                newPath.add(BezierPoint(anchor, prevHandle, nextHandle, true))
            }
            return newPath
        }
    }

    enum class PointType {
        ANCHOR,
        PREV_HANDLE,
        NEXT_HANDLE;

        fun opposite(): PointType {
            return when (this) {
                ANCHOR -> ANCHOR
                PREV_HANDLE -> NEXT_HANDLE
                NEXT_HANDLE -> PREV_HANDLE
            }
        }
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

    fun updateHandles(type: PointType, newPoint: Vec2d) {
        if (type == PointType.ANCHOR) {
            // it's called updateHandles and not updatePoints/updateAnchor for a reason
            throw IllegalArgumentException("Do not call updateHandle with an anchor point")
        }
        modified = true
        if (!split && !mirrored) {
            val oppositeType = type.opposite()
            val oppositePoint = getType(oppositeType)
            if (oppositePoint != null) {
                val distance = oppositePoint.distanceTo(anchor)
                val angle = newPoint.angleTo(anchor)
                val newOppositePoint = Vec2d(distance, 0.0).rotate(angle) + anchor
                setType(oppositeType, newOppositePoint)
            }
        }
        setType(type, newPoint)
        if (split || !mirrored) {
            return
        }
        val oppositeType = type.opposite()
        if (getType(oppositeType) == null) {
            return
        }
        val newDiff = newPoint - anchor
        setType(oppositeType, anchor - newDiff)
        return
    }

    fun draw(g2d: Graphics2D, ppi: Double, scale: Double, prevColor: Color, nextColor: Color) {
        // draw the previous handle
        val size = 2.0 * scale
        val outlineWidth = 0.2 * scale
        val handleWidth = 0.2 * scale
        for (type in listOf(PointType.PREV_HANDLE, PointType.NEXT_HANDLE)) {
            val handle = getType(type) ?: continue
            val color = when (type) {
                PointType.PREV_HANDLE -> prevColor
                PointType.NEXT_HANDLE -> nextColor
                else -> continue
            }
            g2d.color = color
            g2d.stroke = BasicStroke((handleWidth * ppi).toFloat())
            Utils.drawLine(g2d, ppi, anchor, handle)
            g2d.color = g2d.background
            g2d.stroke = BasicStroke((outlineWidth * ppi).toFloat())
            Utils.drawPoint(g2d, ppi, handle, size)
            g2d.color = color
            Utils.drawPoint(g2d, ppi, handle, size, false)
        }

        // draw anchor point
        g2d.color = g2d.background
        Utils.drawPoint(g2d, ppi, anchor, size)
        g2d.color = Color.WHITE
        g2d.stroke = BasicStroke((outlineWidth * ppi).toFloat())
        Utils.drawPoint(g2d, ppi, anchor, size, false)
    }

    fun copy(): BezierPoint {
        return BezierPoint(anchor.copy(), prevHandle?.copy(), nextHandle?.copy(), modified, mirrored, split)
    }

    override fun toString(): String {
        return "BezierPoint(anchor=$anchor, prev=$prevHandle, next=$nextHandle, mod=$modified, mir=$mirrored, split=$split)"
    }
}