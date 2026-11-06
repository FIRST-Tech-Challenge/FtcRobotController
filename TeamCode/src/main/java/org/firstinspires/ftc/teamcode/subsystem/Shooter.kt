package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.follower.Follower
import com.pedropathing.math.Vector2D
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystem.turret.CRTurret
import org.firstinspires.ftc.teamcode.subsystem.turret.Turret
import kotlin.math.abs

class Shooter(
    val taskManager: TaskManager,
    val follower: Follower,
    val outtake: Outtake,
    val turret: Turret,
    val gate: Gate
) : Subsystem() {
    enum class Mode {
        FULL_AUTO,
        AUTO_TARGET,
        MANUAL,
        OFF
    }
    companion object {
        fun launchSpeedFromDistanceSquared(distance: Double): Double {
            TODO("Not yet implemented")
        }
        fun outtakeVelocityFromVelocityVector(velocity: Vector2D): Double {
            val speedSquared = velocity.magnitudeSquared()
            TODO("Not yet implemented")
        }
        fun turretAngleFromVelocityVector(velocityVector: Vector2D): Double {
            val angleRadians = velocityVector.theta()
            return Math.toDegrees(angleRadians)
        }
        const val MAXIMUM_ATTACK_ANGLE: Double = 60.0
    }
    init {
        taskManager.loop.action += { tick() }
    }
    var mode: Mode = Mode.FULL_AUTO
        set(mode) {
            field = mode
            gate.close()
            when (mode) {
                Mode.FULL_AUTO -> {}
                Mode.AUTO_TARGET -> {}
                Mode.MANUAL -> {}
                Mode.OFF -> {
                    outtake.off()
                    turret.off()
                }
            }
        }
    val canReachHive: Boolean
        get() {
            val relativePosition = Alliance.targetHive.position - follower.pose().toVector2D()
            val attackAngle = AngleUnit.normalizeDegrees(Math.toDegrees(relativePosition.theta()) - Alliance.targetHive.outwardsAngle)
            return abs(attackAngle) < Math.toRadians(MAXIMUM_ATTACK_ANGLE)
        }
    val launchVelocityVector: Vector2D
        get() {
            val relativePosition = Alliance.targetHive.position - follower.pose().toVector2D()
            val speed = launchSpeedFromDistanceSquared(relativePosition.magnitudeSquared())
            val absoluteVelocity = relativePosition * (speed / relativePosition.magnitude())
            return absoluteVelocity - follower.velocity().toVector2D()
        }

    fun tick() {
        when (mode) {
            Mode.FULL_AUTO -> {
                target()
                when (canReachHive && outtake.isWithinTolerance && turret.isWithinTolerance) {
                    true -> gate.open()
                    false -> gate.close()
                }
            }
            Mode.AUTO_TARGET -> {
                target()
            }
            else -> {}
        }
    }

    private fun target() {
        turret.angle = turretAngleFromVelocityVector(launchVelocityVector)
        outtake.velocity = outtakeVelocityFromVelocityVector(launchVelocityVector)
    }
    override fun printState(telemetry: Telemetry) {
        TODO("Not yet implemented")
    }

}