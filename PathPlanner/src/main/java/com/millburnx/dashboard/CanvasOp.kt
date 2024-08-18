package com.millburnx.dashboard

import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Font
import java.awt.Graphics2D
import java.awt.geom.AffineTransform
import java.io.File
import javax.imageio.ImageIO
import javax.swing.JPanel

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

    // max for centering
    abstract fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel)
}

// https://github.com/acmerobotics/ftc-dashboard/tree/master/DashboardCore/src/main/java/com/acmerobotics/dashboard/canvas
class Alpha(private val alpha: Double) : CanvasOp(Type.ALPHA) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val prevColor = g2d.color
        g2d.color = Color(prevColor.red, prevColor.green, prevColor.blue, (alpha * 255).toInt())
    }
}

class Circle(
    val x: Double,
    val y: Double,
    private val radius: Double,
    private val stroke: Boolean
) : CanvasOp(
    Type.CIRCLE
) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        Utils.drawPoint(g2d, ppi, Vec2d(x, y), radius * 2, !stroke)
    }
}

class Fill(private val color: String) : CanvasOp(Type.FILL) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val color = Color.decode(color)
        val alpha = g2d.color.alpha
        g2d.color = Color(color.red, color.green, color.blue, alpha)
    }
}

class Grid(
    val x: Double,
    val y: Double,
    private val width: Double,
    private val height: Double,
    private val numTicksX: Int,
    private val numTicksY: Int,
    private val theta: Double,
    private val pivotX: Double,
    private val pivotY: Double,
    private val usePageFrame: Boolean
) : CanvasOp(Type.GRID) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val ticksX = numTicksX - 1
        val ticksY = numTicksY - 1
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
            g2d.translate((panel.width - 144 * ppi) / 2, (panel.height - 144 * ppi) / 2) // center in screen
        }
        g2d.rotate(theta, pivotX * ppi, pivotY * ppi)
        for (i in 0..ticksX) {
            val x = this.x + i * width / ticksX
            Utils.drawLine(g2d, ppi, Vec2d(x, this.y), Vec2d(x, this.y + height))
        }
        for (i in 0..ticksY) {
            val y = this.y + i * height / ticksY
            Utils.drawLine(g2d, ppi, Vec2d(this.x, y), Vec2d(this.x + width, y))
        }
        g2d.transform = prevTransform
    }
}

class Image(
    private val path: String,
    val x: Double,
    val y: Double,
    private val width: Double,
    private val height: Double,
    private val theta: Double,
    private val pivotX: Double,
    private val pivotY: Double,
    private val usePageFrame: Boolean
) : CanvasOp(Type.IMAGE) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
            g2d.translate((panel.width - 144 * ppi) / 2, (panel.height - 144 * ppi) / 2) // center in screen
        }
        g2d.rotate(theta, pivotX * ppi, pivotY * ppi)
        val image = ImageIO.read(File(path))
        g2d.drawImage(image, (x * ppi).toInt(), (y * ppi).toInt(), (width * ppi).toInt(), (height * ppi).toInt(), null)
        g2d.transform = prevTransform
    }
}

class Polygon(private val xPoints: DoubleArray, private val yPoints: DoubleArray, private val stroke: Boolean) :
    CanvasOp(Type.POLYGON) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val xPointsScaled = xPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        val yPointsScaled = yPoints.map { it * ppi }.map { it.toInt() }.toIntArray()
        if (stroke) {
            g2d.drawPolygon(xPointsScaled, yPointsScaled, xPoints.size)
        } else {
            g2d.fillPolygon(xPointsScaled, yPointsScaled, xPoints.size)
        }
    }
}

class Polyline(private val xPoints: DoubleArray, private val yPoints: DoubleArray) :
    CanvasOp(Type.POLYLINE) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
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

class Rotation(private val rotation: Double) : CanvasOp(Type.ROTATION) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        g2d.rotate(rotation)
    }
}

class Scale(private val scaleX: Double, private val scaleY: Double) : CanvasOp(Type.SCALE) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        g2d.scale(scaleX, scaleY)
    }
}

class Stroke(private val color: String) : CanvasOp(Type.STROKE) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val color = Color.decode(color)
        val alpha = g2d.color.alpha
        g2d.color = Color(color.red, color.green, color.blue, alpha)
    }
}

class StrokeWidth(private val width: Int) : CanvasOp(Type.STROKE_WIDTH) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        g2d.stroke = BasicStroke(width.toFloat())
    }
}

class Text(
    private val text: String,
    val x: Double,
    val y: Double,
    private val font: String,
    private val theta: Double,
    val stroke: Boolean,
    private val usePageFrame: Boolean
) : CanvasOp(Type.TEXT) {
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        val prevTransform = g2d.transform
        if (usePageFrame) {
            g2d.transform = AffineTransform()
            g2d.translate((panel.width - 144 * ppi) / 2, (panel.height - 144 * ppi) / 2) // center in screen
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
    override fun draw(g2d: Graphics2D, ppi: Double, panel: JPanel) {
        g2d.translate(x * ppi, y * ppi)
    }
}