package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.millburnx.purepursuit.PurePursuit
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.common.subsystems.PID
import org.firstinspires.ftc.teamcode.common.utils.Util

class PurePursuitCommand(
    val drive: DriveSubsystem,
    val path: List<Vec2d>,
    val lookahead: Double = 14.0,
) : CommandBase() {
    val purePursuit = PurePursuit(path, lookahead)
    val pidX = PID(1.0, 0.0, 0.0)
    val pidY = PID(1.0, 0.0, 0.0)
    val pidH = PID(1.0, 0.0, 0.0)

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        pidX.reset()
        pidY.reset()
        pidH.reset()
    }

    override fun execute() {
        val pose = drive.pos
        val position = Vec2d(pose.x, pose.y)
        val heading = pose.heading
        val targetPoint = purePursuit.calc(position, heading).target

        val powerX = pidX.calc(targetPoint.x, position.x)
        val powerY = pidX.calc(targetPoint.y, position.y)
        val angleDiff = Util.getAngleDiff((position to heading), targetPoint)
        val powerH = pidX.calc(angleDiff, heading)

        drive.robotCentric(powerX, powerY, powerH)
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return purePursuit.isFinished
    }
}