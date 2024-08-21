package com.millburnx.pathplanner

import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Vec2d
import java.awt.event.KeyAdapter
import java.awt.event.KeyEvent
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent

class PathPlannerListeners(pathPlanner: PathPlanner) {
    var selectedPoint: Pair<BezierPoint, BezierPoint.PointType>? = null
    var selectedCopy: BezierPoint? = null

    inner class Mouse(val pathPlanner: PathPlanner) : MouseAdapter() {
        override fun mousePressed(e: MouseEvent) {
            super.mousePressed(e)
            val clickPoint = Vec2d(e.x, e.y)
            val fieldSize = Vec2d(pathPlanner.width, pathPlanner.height)
            val fieldPoint = (clickPoint - fieldSize / 2) / pathPlanner.ppi
            val thresholds = Pair(
                10 / pathPlanner.ppi * pathPlanner.scale,
                10 / pathPlanner.ppi * pathPlanner.scale
            )

            when (e.button) {
                MouseEvent.BUTTON1 -> {
                    if (pathPlanner.currentPopoverRef != null) {
                        pathPlanner.currentPopoverRef?.closePopover()
                        pathPlanner.currentPopoverRef = null
                        return
                    }
                    val targetPoint = BezierPoint.selectedPoint(
                        pathPlanner.bezierPoints, fieldPoint, listOf(
                            BezierPoint.PointType.ANCHOR,
                            BezierPoint.PointType.PREV_HANDLE,
                            BezierPoint.PointType.NEXT_HANDLE
                        ), thresholds
                    )
                    selectedPoint = targetPoint
                    selectedCopy = targetPoint?.first?.copy()
                    println("Selected point: $selectedPoint")
                    if (targetPoint == null) {
                        pathPlanner.addPoint(BezierPoint(fieldPoint))
                    }
                }

                MouseEvent.BUTTON3 -> {
                    if (pathPlanner.currentPopoverRef != null) {
                        pathPlanner.currentPopoverRef?.closePopover()
                        pathPlanner.currentPopoverRef = null
                        return
                    }
                    val targetPoint = BezierPoint.selectedPoint(
                        pathPlanner.bezierPoints, fieldPoint, listOf(
                            BezierPoint.PointType.ANCHOR
                        ), thresholds
                    )
                    if (targetPoint != null) {
                        pathPlanner.removePoint(targetPoint.first)
                    }
                }

                MouseEvent.BUTTON2 -> {
                    if (pathPlanner.currentPopoverRef != null) {
                        pathPlanner.currentPopoverRef?.closePopover()
                        pathPlanner.currentPopoverRef = null
                    }
                    val targetPoint = BezierPoint.selectedPoint(
                        pathPlanner.bezierPoints, fieldPoint, listOf(
                            BezierPoint.PointType.ANCHOR
                        ), thresholds
                    )
                    if (targetPoint != null) {
                        pathPlanner.addPopover(clickPoint, targetPoint)
                    }
                }
            }
        }

        override fun mouseReleased(e: MouseEvent) {
            if (selectedPoint != null) {
                val type = selectedPoint!!.second
                val point = selectedPoint!!.first
                val copy = selectedCopy!!
                val modification = PointModification(
                    point,
                    if (point.modified != copy.modified) point.modified else null,
                    if (point.mirrored != copy.mirrored) point.mirrored else null,
                    if (point.split != copy.split) point.split else null
                )
                val diff = point.getType(type)!! - copy.getType(type)!!
                val change = PointTranslation(point, type, diff)
                pathPlanner.addChanges(listOf(modification, change))
                selectedPoint = null
                selectedCopy = null
            }
        }
    }

    inner class MouseMotion(val pathPlanner: PathPlanner) : MouseAdapter() {
        override fun mouseDragged(e: MouseEvent) {
            if (selectedPoint == null) return super.mouseDragged(e)
            val clickPoint = Vec2d(e.x, e.y)
            val fieldSize = Vec2d(pathPlanner.width, pathPlanner.height)
            val fieldPoint = (clickPoint - fieldSize / 2) / pathPlanner.ppi

            val bezierPoint = selectedPoint!!.first
            val pointType = selectedPoint!!.second

            if (pointType != BezierPoint.PointType.ANCHOR) {
                bezierPoint.modified = true
                if (e.isShiftDown) {
                    bezierPoint.mirrored = false
                }
                if (e.isAltDown) {
                    bezierPoint.split = true
                }
            }
            bezierPoint.updateType(pointType, fieldPoint)
            pathPlanner.updateCatmullRom()
            pathPlanner.repaint()
        }
    }

    class Key(val pathPlanner: PathPlanner) : KeyAdapter() {
        override fun keyPressed(e: KeyEvent) {
            when (e.keyCode) {
                KeyEvent.VK_Z -> {
                    if (!e.isControlDown) return
                    if (e.isShiftDown) {
                        pathPlanner.redo()
                    } else {
                        pathPlanner.undo()
                    }
                }
            }
        }
    }

    val mouse = Mouse(pathPlanner)
    val mouseMotion = MouseMotion(pathPlanner)
    val key = Key(pathPlanner)
}