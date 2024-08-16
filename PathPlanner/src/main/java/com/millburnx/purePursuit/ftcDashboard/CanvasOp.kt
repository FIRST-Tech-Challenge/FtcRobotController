package com.millburnx.purePursuit.ftcDashboard

import com.millburnx.purePursuit.Utils.Point
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Font
import java.awt.Graphics2D
import java.awt.geom.AffineTransform
import java.io.File
import javax.imageio.ImageIO

// https://acmerobotics.github.io/ftc-dashboard/javadoc/com/acmerobotics/dashboard/canvas/CanvasOp.html
abstract class CanvasOp(val type: Type) {
    enum class Type {
        GRID,
        TRANSLATE,
        ROTATION,
        SCALE,
        ALPHA,
        CIRCLE,
        POLYGON,
        POLYLINE,
        STROKE,
        FILL,
        STROKE_WIDTH,
        TEXT,
        IMAGE
    }

    abstract fun draw(g2d: Graphics2D, ppi: Double)
}

// https://github.com/acmerobotics/ftc-dashboard/tree/master/DashboardCore/src/main/java/com/acmerobotics/dashboard/canvas
class Alpha(val alpha: Double) : CanvasOp(Type.ALPHA) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val prevColor = g2d.color
        g2d.color = Color(prevColor.red, prevColor.green, prevColor.blue, (alpha * 255).toInt())
    }
}

class Circle(
    val x: Double,
    val y: Double,
    val radius: Double,
    val stroke: Boolean
) : CanvasOp(
    Type.CIRCLE
) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val center = Point(x, y) * ppi
        val r = radius * ppi
        val origin = center - Point(r, r)
        if (stroke) {
            g2d.drawOval(origin.x.toInt(), origin.y.toInt(), (2 * r).toInt(), (2 * r).toInt())
        } else {
            g2d.fillOval(origin.x.toInt(), origin.y.toInt(), (2 * r).toInt(), (2 * r).toInt())
        }
    }
}

class Fill(val color: String) : CanvasOp(Type.FILL) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val color = Color.decode(color)
        val alpha = g2d.color.alpha
        g2d.color = Color(color.red, color.green, color.blue, alpha)
    }
}

class Grid(
    val x: Double,
    val y: Double,
    val width: Double,
    val height: Double,
    val numTicksX: Int,
    val numTicksY: Int,
    val theta: Double,
    val pivotX: Double,
    val pivotY: Double,
    val usePageFrame: Boolean
) : CanvasOp(Type.GRID) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val ticksX = numTicksX - 1
        val ticksY = numTicksY - 1
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
        }
        g2d.rotate(theta, pivotX * ppi, pivotY * ppi)
        for (i in 0..ticksX) {
            val x = this.x + i * width / ticksX
            g2d.drawLine(
                (x * ppi).toInt(),
                (this.y * ppi).toInt(),
                (x * ppi).toInt(),
                ((this.y + height) * ppi).toInt()
            )
        }
        for (i in 0..ticksY) {
            val y = this.y + i * height / ticksY
            g2d.drawLine((this.x * ppi).toInt(), (y * ppi).toInt(), ((this.x + width) * ppi).toInt(), (y * ppi).toInt())
        }
        g2d.transform = prevTransform
    }
}

class Image(
    val path: String,
    val x: Double,
    val y: Double,
    val width: Double,
    val height: Double,
    val theta: Double,
    val pivotX: Double,
    val pivotY: Double,
    val usePageFrame: Boolean
) : CanvasOp(Type.IMAGE) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
        }
        g2d.rotate(theta, pivotX * ppi, pivotY * ppi)
        val image = ImageIO.read(File(path))
        g2d.drawImage(image, (x * ppi).toInt(), (y * ppi).toInt(), (width * ppi).toInt(), (height * ppi).toInt(), null)
        g2d.transform = prevTransform
    }
}

class Polygon(val xPoints: DoubleArray, val yPoints: DoubleArray, val stroke: Boolean) :
    CanvasOp(Type.POLYGON) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val xPointsScaled = xPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        val yPointsScaled = yPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        if (stroke) {
            g2d.drawPolygon(xPointsScaled, yPointsScaled, xPoints.size)
        } else {
            g2d.fillPolygon(xPointsScaled, yPointsScaled, xPoints.size)
        }
    }
}

class Polyline(val xPoints: DoubleArray, val yPoints: DoubleArray) :
    CanvasOp(Type.POLYLINE) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        if (xPoints.size != yPoints.size) {
            throw IllegalArgumentException("xPoints and yPoints must have the same length")
        }
        if (xPoints.isEmpty()) {
            return
        }
        val xPointsScaled = xPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        val yPointsScaled = yPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        g2d.drawPolyline(xPointsScaled, yPointsScaled, xPoints.size)
    }
}

class Rotation(val rotation: Double) : CanvasOp(Type.ROTATION) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        g2d.rotate(rotation)
    }
}

class Scale(val scaleX: Double, val scaleY: Double) : CanvasOp(Type.SCALE) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        g2d.scale(scaleX, scaleY)
    }
}

class Stroke(val color: String) : CanvasOp(Type.STROKE) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val color = Color.decode(color)
        val alpha = g2d.color.alpha
        g2d.color = Color(color.red, color.green, color.blue, alpha)
    }
}

class StrokeWidth(val width: Int) : CanvasOp(Type.STROKE_WIDTH) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        g2d.stroke = BasicStroke(width.toFloat())
    }
}

class Text(
    val text: String,
    val x: Double,
    val y: Double,
    val font: String,
    val theta: Double,
    val stroke: Boolean,
    val usePageFrame: Boolean
) : CanvasOp(Type.TEXT) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
        }
        g2d.rotate(theta, x * ppi, y * ppi)
        val fontSize = font.split(" ")[0].slice(0..<font.split(" ")[0].length - 2).toDouble()
        val fontName = font.split(" ").slice(1..<font.split(" ").size).joinToString(" ")
        val font = Font(fontName, Font.PLAIN, (fontSize * ppi).toInt())
        g2d.font = font
        g2d.drawString(text, (x * ppi).toInt(), (y * ppi).toInt())
        g2d.transform = prevTransform
    }
}

class Translate(val x: Double, val y: Double) : CanvasOp(Type.TRANSLATE) {
    override fun draw(g2d: Graphics2D, ppi: Double) {
        g2d.translate(x * ppi, y * ppi)
    }
}