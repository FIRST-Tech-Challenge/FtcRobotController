package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Twist
import com.pedropathing.math.Velocity
import org.firstinspires.ftc.teamcode.subsystem.FieldDrawing

class PlottingLocalizer(val pinpoint: Localizer): Localizer {

    val fieldDrawing = FieldDrawing()

    override fun setPose(p0: Pose?) {
        pinpoint.setPose(p0)
    }

    override fun setX(x: Double) {
        pinpoint.setX(x)
    }

    override fun setY(y: Double) {
        pinpoint.setY(y)
    }

    override fun setHeading(heading: Double) {
        pinpoint.setHeading(heading)
    }

    override fun pose(): Pose? {
        return pinpoint.pose()
    }

    override fun twist(): Twist? {
        return pinpoint.twist()
    }

    override fun velocity(): Velocity? {
        return pinpoint.velocity()
    }

    override fun state(): MotionState? {
        return pinpoint.state()
    }

    override fun update() {
        pinpoint.update()
        fieldDrawing.drawRobot(pinpoint.pose(), 0.0)
    }

    override fun reset() {
        pinpoint.reset()
    }

    override fun debug(): Map<String?, Any?>? {
        return pinpoint.debug()
    }
}