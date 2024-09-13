package com.millburnx.utils

import java.awt.BasicStroke
import java.awt.Color
import java.awt.FileDialog
import java.awt.Font
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.Toolkit
import java.awt.image.BufferedImage
import java.io.File
import javax.swing.JLabel
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sin

class Utils {
    class Colors {
        companion object {
            // From Freya Holmér
            val bg0 = "#070b0f"
            val bg1 = "#0e1a25"
            val bg2 = "#525c65"
            val bg3 = "#4f6880"
            val bg4 = "#a6aeb6"

            val red = "#fb1155"
            val blue = "#35b8fa"
            val green = "#11e59c"
            val yellow = "#f1d454"

            val purple = "#8e10fd"
        }
    }

    companion object {
        /**
         * Normalize an angle to be between -pi and pi radians (-180 to 180 degrees)
         */
        fun normalizeAngle(angle: Double): Double {
            return atan2(sin(angle), cos(angle))
        }

        /**
         * Converts a list of points to a list of Béziers
         */
        fun pathToBeziers(path: List<Vec2d>): MutableList<Bezier> {
            val beziers = mutableListOf<Bezier>()
            for (i in 0 until path.size - 3 step 3) {
                beziers.add(Bezier(path[i], path[i + 1], path[i + 2], path[i + 3]))
            }
            return beziers
        }

        fun drawPoint(g2d: Graphics2D, ppi: Double, point: Vec2d, size: Double, filled: Boolean = true) {
            val center = point * ppi
            val size = size * ppi
            val origin = center - size / 2
            if (filled) {
                g2d.fillOval(origin.x.toInt(), origin.y.toInt(), size.toInt(), size.toInt())
            } else {
                g2d.drawOval(origin.x.toInt(), origin.y.toInt(), size.toInt(), size.toInt())
            }
        }

        fun drawLine(g2d: Graphics2D, ppi: Double, start: Vec2d, end: Vec2d) {
            val start = start * ppi
            val end = end * ppi
            g2d.drawLine(start.x.toInt(), start.y.toInt(), end.x.toInt(), end.y.toInt())
        }

        /**
         * Opens a file dialog to select a file
         * @param directory The base directory to open the dialog in
         * @param file The default file to select (or file types if supported)
         * @param save Whether to open a save dialog or a load dialog
         * @return The selected file
         */
        fun fileDialog(directory: String, file: String, save: Boolean = false): File? {
            val action = if (save) FileDialog.SAVE else FileDialog.LOAD
            val fileDialog = FileDialog(null as java.awt.Frame?, "Select a file", action)
            fileDialog.directory = File(directory).absolutePath
            fileDialog.file = file
            fileDialog.isVisible = true
            val file = fileDialog.file ?: return null
            return File(fileDialog.directory, file)
        }

        /**
         * Creates a buffered image with antialiasing and subpixel rendering enabled
         * @return A pair containing the buffered image and the graphics object
         */
        fun bufferedImage(width: Int, height: Int): Pair<BufferedImage, Graphics2D> {
            val bufferedImage = BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)
            val g2d = bufferedImage.createGraphics()

            val desktopHints = Toolkit.getDefaultToolkit().getDesktopProperty("awt.font.desktophints")

            g2d.setRenderingHints(RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON))
            g2d.addRenderingHints(RenderingHints(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE))
            g2d.addRenderingHints(
                RenderingHints(
                    RenderingHints.KEY_TEXT_ANTIALIASING,
                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON
                )
            )
            desktopHints?.let { g2d.addRenderingHints(desktopHints as RenderingHints) }

            return Pair(bufferedImage, g2d)
        }

        /**
         * Draws a rounded panel with a border
         */
        fun drawRoundedPanel(
            g2d: Graphics2D, scale: Double, size: Vec2d, bgColor: Color, radius: Double,
            borderColor: Color = Color.decode(Colors.bg2), borderWidth: Float = 0f
        ) {
            val radius = (radius * scale).toInt()

            g2d.color = bgColor
            g2d.fillRoundRect(0, 0, size.x.toInt(), size.y.toInt(), radius, radius)

            if (borderWidth <= 0) return
            val borderWidth = (borderWidth * scale).toFloat()

            g2d.stroke = BasicStroke(borderWidth)
            g2d.color = borderColor
            g2d.drawRoundRect(
                ceil(borderWidth / 2).toInt(),
                ceil(borderWidth / 2).toInt(),
                floor(size.x - borderWidth).toInt(),
                floor(size.y - borderWidth).toInt(),
                radius,
                radius
            )
        }

        /**
         * Creates a JLabel with the specified text, font, and color
         */
        fun jLabel(text: String, font: Font, color: Color): JLabel {
            val label = JLabel(text)
            label.font = font
            label.foreground = color
            return label
        }
    }
}