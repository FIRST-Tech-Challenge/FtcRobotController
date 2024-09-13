package com.millburnx.dashboard

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import java.awt.Graphics2D
import javax.swing.JPanel

abstract class CanvasOp(val type: Type) : com.acmerobotics.dashboard.canvas.CanvasOp(type) {
    abstract fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel)
}

class Canvas : com.acmerobotics.dashboard.canvas.Canvas() {
    private val ops = mutableListOf<CanvasOp>()

    override fun setScale(scaleX: Double, scaleY: Double): Canvas {
        this.ops.add(Scale(scaleX, scaleY))
        return this
    }

    override fun setRotation(radians: Double): Canvas {
        this.ops.add(Rotation(radians))
        return this
    }

    override fun setTranslation(x: Double, y: Double): Canvas {
        this.ops.add(Translate(x, y))
        return this
    }

    override fun strokeText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean
    ): Canvas {
        this.ops.add(Text(text, x, y, font, theta, true, usePageFrame))
        return this
    }

    override fun strokeText(text: String, x: Double, y: Double, font: String, theta: Double): Canvas {
        this.strokeText(text, x, y, font, theta, true)
        return this
    }

    override fun fillText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean
    ): Canvas {
        this.ops.add(Text(text, x, y, font, theta, false, usePageFrame))
        return this
    }

    override fun fillText(text: String, x: Double, y: Double, font: String, theta: Double): Canvas {
        this.fillText(text, x, y, font, theta, true)
        return this
    }

    override fun strokeCircle(x: Double, y: Double, radius: Double): Canvas {
        this.ops.add(Circle(x, y, radius, true))
        return this
    }

    override fun fillCircle(x: Double, y: Double, radius: Double): Canvas {
        this.ops.add(Circle(x, y, radius, false))
        return this
    }

    override fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        this.ops.add(Polygon(xPoints, yPoints, true))
        return this
    }

    override fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        this.ops.add(Polygon(xPoints, yPoints, false))
        return this
    }

    override fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        this.ops.add(Polyline(xPoints, yPoints))
        return this
    }

    override fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): Canvas {
        this.strokePolyline(doubleArrayOf(x1, x2), doubleArrayOf(y1, y2))
        return this
    }

    override fun fillRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        this.fillPolygon(doubleArrayOf(x, x + width, x + width, x), doubleArrayOf(y, y, y + height, y + height))
        return this
    }

    override fun strokeRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        this.strokePolygon(doubleArrayOf(x, x + width, x + width, x), doubleArrayOf(y, y, y + height, y + height))
        return this
    }

    override fun setFill(color: String): Canvas {
        this.ops.add(Fill(color))
        return this
    }

    override fun setStroke(color: String): Canvas {
        this.ops.add(Stroke(color))
        return this
    }

    override fun setStrokeWidth(width: Int): Canvas {
        this.ops.add(StrokeWidth(width))
        return this
    }

    override fun drawImage(image: String, x: Double, y: Double, width: Double, height: Double): Canvas {
        this.drawImage(image, x, y, width, height, 0.0, 0.0, 0.0, true)
        return this
    }

    override fun drawImage(
        image: String,
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        theta: Double,
        pivotX: Double,
        pivotY: Double,
        usePageFrame: Boolean
    ): Canvas {
        this.ops.add(Image(image, x, y, width, height, theta, pivotX, pivotY, usePageFrame))
        return this
    }

    override fun drawGrid(x: Double, y: Double, width: Double, height: Double, numTicksX: Int, numTicksY: Int): Canvas {
        this.drawGrid(x, y, width, height, numTicksX, numTicksY, 0.0, 0.0, 0.0, true)
        return this
    }

    override fun drawGrid(
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        numTicksX: Int,
        numTicksY: Int,
        theta: Double,
        pivotX: Double,
        pivotY: Double,
        usePageFrame: Boolean
    ): Canvas {
        this.ops.add(Grid(x, y, width, height, numTicksX, numTicksY, theta, pivotX, pivotY, usePageFrame))
        return this
    }

    override fun setAlpha(alpha: Double): Canvas {
        this.ops.add(Alpha(alpha))
        return this
    }

    override fun getOperations(): List<CanvasOp> {
        return this.ops
    }

    override fun clear() {
        this.ops.clear()
    }
}

class TelemetryPacket : TelemetryPacket() {
    private val data = sortedMapOf<String, Any>()
    private val lines = mutableListOf<String>()
    internal val timestamp: Long? = null
    private val canvas: Canvas = Canvas()

    override fun addLine(line: String) {
        lines.add(line)
    }

    override fun put(key: String, value: Any) {
        data[key] = value
    }

    override fun putAll(map: Map<String, Any>) {
        data.putAll(map)
    }

    override fun clearLines() {
        lines.clear()
    }

    override fun addTimestamp(): Long {
        return System.currentTimeMillis()
    }

    override fun fieldOverlay(): Canvas {
        return canvas
    }
}