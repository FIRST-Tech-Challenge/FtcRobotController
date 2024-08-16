package com.millburnx.purePursuit.ftcDashboard

// https://acmerobotics.github.io/ftc-dashboard/javadoc/com/acmerobotics/dashboard/canvas/Canvas.html
interface ICanvas {
    fun clear()
    fun drawGrid(
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        numTicksX: Int,
        numTicksY: Int,
        theta: Double = 0.0,
        pivotX: Double = 0.0,
        pivotY: Double = 0.0,
        usePageFrame: Boolean = true
    ): ICanvas

    fun drawImage(
        path: String,
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        theta: Double = 0.0,
        pivotX: Double = 0.0,
        pivotY: Double = 0.0,
        usePageFrame: Boolean = true
    ): ICanvas

    fun fillCircle(x: Double, y: Double, radius: Double): ICanvas
    fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): ICanvas
    fun fillRect(x: Double, y: Double, width: Double, height: Double): ICanvas
    fun fillText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean = true
    ): ICanvas

    fun getOperations(): List<CanvasOp>
    fun setAlpha(alpha: Double): ICanvas
    fun setFill(color: String): ICanvas
    fun setRotation(radians: Double): ICanvas
    fun setScale(scaleX: Double, scaleY: Double): ICanvas
    fun setStroke(color: String): ICanvas
    fun setStrokeWidth(width: Int): ICanvas
    fun setTranslation(x: Double, y: Double): ICanvas
    fun strokeCircle(x: Double, y: Double, radius: Double): ICanvas
    fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): ICanvas
    fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): ICanvas
    fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): ICanvas
    fun strokeRect(x: Double, y: Double, width: Double, height: Double): ICanvas
    fun strokeText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean = true
    ): ICanvas
}

// https://github.com/acmerobotics/ftc-dashboard/blob/master/DashboardCore/src/main/java/com/acmerobotics/dashboard/canvas/Canvas.java
class Canvas : ICanvas {
    private val ops: MutableList<CanvasOp> = ArrayList()

    override fun setScale(scaleX: Double, scaleY: Double): Canvas {
        ops.add(Scale(scaleX, scaleY))
        return this
    }

    override fun setRotation(radians: Double): Canvas {
        ops.add(Rotation(radians))
        return this
    }

    override fun setTranslation(x: Double, y: Double): Canvas {
        ops.add(Translate(x, y))
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
        ops.add(Text(text, x, y, font, theta, true, usePageFrame))
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
        ops.add(Text(text, x, y, font, theta, false, usePageFrame))
        return this
    }

    override fun strokeCircle(x: Double, y: Double, radius: Double): Canvas {
        ops.add(Circle(x, y, radius, true))
        return this
    }

    override fun fillCircle(x: Double, y: Double, radius: Double): Canvas {
        ops.add(Circle(x, y, radius, false))
        return this
    }

    override fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): ICanvas {
        ops.add(Polygon(xPoints, yPoints, false))
        return this
    }

    override fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        ops.add(Polygon(xPoints, yPoints, true))
        return this
    }

    override fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        ops.add(Polyline(xPoints, yPoints))
        return this
    }

    override fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): Canvas {
        strokePolyline(doubleArrayOf(x1, x2), doubleArrayOf(y1, y2))
        return this
    }

    override fun fillRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        fillPolygon(
            doubleArrayOf(x, x + width, x + width, x),
            doubleArrayOf(y, y, y + height, y + height)
        )
        return this
    }

    override fun strokeRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        strokePolygon(
            doubleArrayOf(x, x + width, x + width, x),
            doubleArrayOf(y, y, y + height, y + height)
        )
        return this
    }

    override fun setFill(color: String): Canvas {
        ops.add(Fill(color))
        return this
    }

    override fun setStroke(color: String): Canvas {
        ops.add(Stroke(color))
        return this
    }

    override fun setStrokeWidth(width: Int): Canvas {
        ops.add(StrokeWidth(width))
        return this
    }

    override fun drawImage(
        path: String, x: Double, y: Double, width: Double, height: Double,
        theta: Double, pivotX: Double, pivotY: Double, usePageFrame: Boolean
    ): Canvas {
        ops.add(Image(path, x, y, width, height, theta, pivotX, pivotY, usePageFrame))
        return this
    }

    override fun drawGrid(
        x: Double, y: Double, width: Double, height: Double, numTicksX: Int,
        numTicksY: Int, theta: Double, pivotX: Double, pivotY: Double,
        usePageFrame: Boolean
    ): Canvas {
        ops.add(
            Grid(
                x, y, width, height, numTicksX, numTicksY, theta, pivotX, pivotY,
                usePageFrame
            )
        )
        return this
    }

    override fun setAlpha(alpha: Double): Canvas {
        ops.add(Alpha(alpha))
        return this
    }

    override fun getOperations(): List<CanvasOp> {
        return ops
    }

    override fun clear() {
        ops.clear()
    }
}