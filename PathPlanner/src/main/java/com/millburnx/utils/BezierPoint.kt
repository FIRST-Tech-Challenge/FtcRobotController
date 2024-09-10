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
        const val pointSize = 2.0
        const val outlineWidth = 0.2
        const val handleWidth = 0.2

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

        /**
         * Return the closest point to the click point within the threshold
         * @param points The list of points to search
         * @param clickPoint The clickPoint
         * @param filter The list of point types to search for
         * @param thresholds A pair containing the anchor and handle thresholds
         */
        fun selectedPoint(
            points: List<BezierPoint>,
            clickPoint: Vec2d,
            filter: List<PointType>,
            thresholds: Pair<Double, Double>
        ): Pair<BezierPoint, PointType>? {
            for (bezierPoint in points) {
                for (type in filter) {
                    val threshold = when (type) {
                        PointType.ANCHOR -> thresholds.first
                        else -> thresholds.second
                    } + pointSize / 2 + outlineWidth
                    val point = bezierPoint.getType(type) ?: continue
                    if (point.distanceTo(clickPoint) < threshold) {
                        return Pair(bezierPoint, type)
                    }
                }
            }
            return null
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

    fun updateType(pointType: PointType, value: Vec2d) {
        if (pointType == PointType.ANCHOR) {
            val diff = value - anchor
            prevHandle = prevHandle?.plus(diff)
            nextHandle = nextHandle?.plus(diff)
            return setType(pointType, value)
        }
        updateHandles(pointType, value)
    }

    fun updateHandles(type: PointType, newPoint: Vec2d) {
        if (type == PointType.ANCHOR) {
            throw IllegalArgumentException("Do not call updateHandle with an anchor point, use updateType instead")
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
        val point = pointSize * scale
        val outlineWidth = outlineWidth * scale
        val handleWidth = handleWidth * scale
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
            Utils.drawPoint(g2d, ppi, handle, point)
            g2d.color = color
            Utils.drawPoint(g2d, ppi, handle, point, false)
        }

        // draw anchor point
        g2d.color = g2d.background
        Utils.drawPoint(g2d, ppi, anchor, point)
        g2d.color = Color.WHITE
        g2d.stroke = BasicStroke((outlineWidth * ppi).toFloat())
        Utils.drawPoint(g2d, ppi, anchor, point, false)
    }

    fun copy(): BezierPoint {
        return BezierPoint(anchor.copy(), prevHandle?.copy(), nextHandle?.copy(), modified, mirrored, split)
    }

    override fun toString(): String {
        return "BezierPoint(anchor=$anchor, prev=$prevHandle, next=$nextHandle, mod=$modified, mir=$mirrored, split=$split)"
    }
}