package org.firstinspires.ftc.teamcode.baseClasses

class UnitDistance private constructor(mm: Double) {
    val mm: Double = mm

    companion object {
        @JvmStatic private val MM_IN_INCH = 25.4

        @JvmStatic fun mm(mm: Double): UnitDistance {
            return UnitDistance(mm)
        }

        @JvmStatic fun mm(mm: Int): UnitDistance {
            return mm(mm.toDouble())
        }

        @JvmStatic fun cm(cm: Double): UnitDistance {
            return UnitDistance(cm * 10)
        }

        @JvmStatic fun cm(cm: Int): UnitDistance {
            return cm(cm.toDouble())
        }

        @JvmStatic fun m(m: Double): UnitDistance {
            return UnitDistance(m * 1000)
        }

        @JvmStatic fun m(m: Int): UnitDistance {
            return m(m.toDouble())
        }

        @JvmStatic fun inches(inches: Double): UnitDistance {
            return UnitDistance(inches * MM_IN_INCH)
        }

        @JvmStatic fun inches(inches: Int): UnitDistance {
            return inches(inches.toDouble())
        }
    }

    val millimetres: Double get() = mm
    val centimetres: Double get() = mm / 10
    val metres: Double get() = mm / 1000
    val inches: Double get() = mm / MM_IN_INCH
}