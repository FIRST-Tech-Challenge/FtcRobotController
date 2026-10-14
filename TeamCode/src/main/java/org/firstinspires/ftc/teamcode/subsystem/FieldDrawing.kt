package org.firstinspires.ftc.teamcode.subsystem

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.math.Pose
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class FieldDrawing {
    val panelsField = PanelsField.field.apply {
        setOffsets(PanelsField.presets.PEDRO_PATHING)
    }

    fun drawRobot(robotPose: Pose, turretRevolutions: Double) {
        val robotX: Double = robotPose.x()
        val robotY: Double = robotPose.y()
        val robotHeading: Double = robotPose.heading()

        PanelsTelemetry.telemetry.addData("robotX", robotX  )
        PanelsTelemetry.telemetry.addData("robotY", robotY)
        PanelsTelemetry.telemetry.addData("heading", robotHeading)

        panelsField.moveCursor(robotX, robotY) // This says where to draw everything

        // Draw the way that the robot is pointing
        panelsField.setStyle("#ed8796", "#ed8796", 2.0)
        panelsField.line(
            robotX + 6*cos(robotHeading),
            robotY + 6*sin(robotHeading))

        panelsField.moveCursor(robotX, robotY)

        // Draw the way that the turret is pointing
        panelsField.setStyle("#7dc4e4", "#7dc4e4", 1.0)
        panelsField.line(
            robotX + 2*cos(turretRevolutions * (2 * PI)),
            robotY + 2*sin(turretRevolutions * (2 * PI)))

        panelsField.moveCursor(robotX, robotY)

        // Draw the robot itself
        panelsField.setStyle("#8087a2", "#494d64", 1.0)
        panelsField.circle(4.0)
        panelsField.update()
    }
}