package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt


class REmoverImpl {

    var finished = false

    companion object{
        var FKP: Double = 0.1 //0.35
        var FKD: Double = 0.01 //0.02
        var FKI: Double = 0.0 // 0.0005

        //0.4, 0.07, 0.00001
        var SKP: Double = 0.075// 0.5
        var SKD: Double = 0.005 // 0.06
        var SKI: Double = 0.0 // 0.0001

        var WKP: Double = 0.08 // 0.4
        var WKD: Double = 0.007 //0.005
        var WKI: Double = 0.0

        /**
         * Radius of turn, inches
         */
        const val R = 8.25
    }

    //target related
    var target = RobotPose(0.0, 0.0, 0.0)
    var tgtx = target.x
    var tgty = target.y
    var tgta = target.a
    var tempTargetAngle = tgta

    //I sums
    var sumF = 0.0
    var sumS = 0.0
    var sumW = 0.0

    //time related
    lateinit var timeout: ElapsedTime
    lateinit var runtime: ElapsedTime

    var deltaTime = 0.0
    var currentTime = 0.0
    var prevTime = 0.0
    var prevDeltaAll = 0.0

    //unique to each drive
    var maxPower = 1.0
    var stopCond: StopConditions = StopConditions.Default
    var timeoutAt = 1.0
    var farStrafe = false
    var rotateBack = false

    fun init(
        currentPose: RobotPose,
        target: RobotPose,
        maxPower: Double = 1.0,
        stopCond: StopConditions = StopConditions.Default,
        timeoutAt: Double = 1.0,
        farStrafe: Boolean = false,
        rotateBack: Boolean = true,
) {

        finished = false

        val ppx = currentPose.x
        val ppy = currentPose.y
        val ppaRad = currentPose.a

        this.target = target
        tgtx = target.x
        tgty = target.y
        tgta = target.a

        this.maxPower = maxPower
        this.stopCond = stopCond
        this.timeoutAt = timeoutAt
        this.farStrafe = farStrafe
        this.rotateBack = rotateBack

        timeout = ElapsedTime(ElapsedTime.Resolution.SECONDS)
        runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        currentTime = runtime.time()
        prevTime = runtime.time()

        val deltaX = tgtx - ppx
        val deltaY = tgty - ppy

        val tempTargetAngle1 = normalize(atan2(deltaY, deltaX))
        val tempTargetAngle2 = normalize(tempTargetAngle1 + PI)
        Log.i("tempA2", tempTargetAngle2.toString())
        Log.i("tempA1", tempTargetAngle1.toString())
        tgta = normalize(tgta)

        val error1 = angleDifference(tempTargetAngle1, ppaRad) + angleDifference(
            tgta,
            tempTargetAngle1
        )
        val error2 = angleDifference(tempTargetAngle2, ppaRad) + angleDifference(
            tgta,
            tempTargetAngle2
        )

        if (farStrafe) {
            if (error1 <= error2) {
                tempTargetAngle = tempTargetAngle1
            } else if (error2 < error1) {
                tempTargetAngle = tempTargetAngle2
            }
        } else {
            tempTargetAngle = tgta
        }
    }

    fun tick(
        ppx: Double,
        ppy: Double,
        ppaRad: Double,
        velXInch: Double,
        velYInch: Double,
        angVelRad: Double,
        speed: Double,
        hw: GerthaHardwareMap,
    ) {

        if (finished) {
            hw.frontLeft.power = 0.0
            hw.frontRight.power = 0.0
            hw.backLeft.power = 0.0
            hw.backRight.power = 0.0
            return
        }
        currentTime = runtime.time()

        val timeoutTime = timeout.time()

        val deltaX = tgtx - ppx
        val deltaY = tgty - ppy
        var deltaA = tempTargetAngle - ppaRad

        deltaA %= 2 * PI
        if (deltaA > PI) {
            deltaA -= 2 * PI
        } else if (deltaA < -PI) {
            deltaA += 2 * PI
        }

        if (stopCond.evaluate.check(
                deltaX,
                deltaY,
                deltaA,
                speed,
                angVelRad
            ) || timeoutTime > timeoutAt
        ) {
            if (timeoutTime > 1) {
                Log.w(
                    "REMover",
                    "Timed out %s: XYA: %.4f %.4f %.4f; speed: %.4f, angvel: %.4f".format(
                        this,
                        deltaX,
                        deltaY,
                        deltaA,
                        speed,
                        angVelRad
                    )
                )
            } else {
                Log.w("Remover", "finished")
            }
            if (stopCond.stopAtEnd) {
                hw.frontLeft.power = 0.0
                hw.frontRight.power = 0.0
                hw.backLeft.power = 0.0
                hw.backRight.power = 0.0
            }
            finished = true
            return
        }


        val f = cos(ppaRad) * deltaX + sin(ppaRad) * deltaY
        val s = sin(ppaRad) * deltaX - cos(ppaRad) * deltaY
        val w = R * deltaA

        deltaTime = max(currentTime - prevTime, 0.001)

        val vF = cos(ppaRad) * velXInch + sin(ppaRad) * velYInch
        val vS = sin(ppaRad) * velXInch - cos(ppaRad) * velYInch
        val vW = R * angVelRad

        if (abs(f) > 1.5) {
            sumF = 0.0
        } else {
            sumF += f * deltaTime
        }

        if (abs(s) > 1.5) {
            sumS = 0.0
        } else {
            sumS += s * deltaTime
        }

//                if (W < 3) {
//                    sumW = 0.0
//                } else {
//                    sumW += W * deltaTime
//                }

        var tipFactor: Double = 1.0

//                if (abs(f) > tipFearRatio * abs(s)) {
//                    val ratio: Double = abs(s) / abs(f)
//
//                    tipFactor = (tipFKP / FKP) + (ratio * tipFearRatio) * (FKP - tipFKP / FKP)
//                }



        val pf: Double = FKP * f + FKI * sumF - FKD * vF
        val ps: Double = SKP * s + SKI * sumS - SKD * vS
        val pw: Double = (WKP) * w + WKI * sumW - WKD * vW


        val deltaAll = sqrt((f * f) + (s * s) + (w * w))

        if (farStrafe && (hypot(deltaX, deltaY) < 30.0)) {
            if (rotateBack) {
                tempTargetAngle = tgta
            }
        }

        if (abs(deltaAll - prevDeltaAll) > 0.5 || currentTime < 1000) {
            prevDeltaAll = deltaAll
            timeout.reset()
        }

        var pfl = pf + ps - pw
        var pbl = pf - ps - pw
        var pfr = pf - ps + pw
        var pbr = pf + ps + pw


        //rescale the four speeds so the largest is +/- 1
        val greatestPower = max(
            max(abs(pfl), abs(pbl)),
            max(abs(pfr), abs(pbr))
        )

        if (greatestPower > maxPower) {
            val scale = greatestPower / maxPower
            pfl /= scale
            pbl /= scale
            pfr /= scale
            pbr /= scale
        }


        hw.frontLeft.power = pfl
        hw.backLeft.power = pbl
        hw.frontRight.power = pfr
        hw.backRight.power = pbr
        prevTime = currentTime

        return
    }
}


internal interface StopCondition {
    fun check(
        deltaX: Double,
        deltaY: Double,
        deltaA: Double,
        speed: Double,
        angVelocity: Double
    ): Boolean
}

enum class StopConditions(internal val evaluate: StopCondition, val stopAtEnd: Boolean) {
    Default(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 0.5
                    && abs(deltaY) < 0.5
                    && abs(deltaA) < Math.PI / 48
                    && speed < 10
                    && abs(angVelocity) < Math.PI / 4)
        },
        stopAtEnd = true
    ),
    Waypoint(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 6
                    && abs(deltaY) < 6
                    && abs(deltaA) < Math.PI / 4)
        },
        stopAtEnd = false
    ),
    Precision(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 0.25
                    && abs(deltaY) < 0.25
                    && abs(deltaA) < Math.PI / 96
                    && speed < 5
                    && abs(angVelocity) < Math.PI / 8)
        },
        stopAtEnd = true
    ),
}

