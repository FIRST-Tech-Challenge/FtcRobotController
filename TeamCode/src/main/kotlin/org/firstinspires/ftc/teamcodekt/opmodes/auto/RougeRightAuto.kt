package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.util.kt.toIn
import ftc.rogue.blacksmith.util.kt.toRad
import org.firstinspires.ftc.teamcode.AutoData.*
import org.firstinspires.ftc.teamcodekt.components.LiftConfig

@Suppress("RemoveRedundantQualifierName")
class RougeRightAuto : RougeBaseAuto() {
    private var cycleNumber = 0
    private var signalZone = 0

    private lateinit var armPosFunction: () -> Unit
    private lateinit var wristPosFunction: () -> Unit
    override fun go() {
        run();
    }

    fun run(): Unit = with(bot) {
        initHardware()

        Scheduler.beforeEach {
            armPosFunction()
            wristPosFunction()
        }

        signalZone = waitForStartWithVision()
        telemetry.addData("Final signal zone", signalZone)
        telemetry.update()

        Anvil.setUnits(distanceUnit = DistanceUnit.CM)

        val startPose = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
        val startTraj = preload(startPose)

        Anvil.startAsyncAutoWith(startTraj)

        Scheduler.launch(this@RougeRightAuto) {
            lift.update(telemetry)
            wrist.update()
            drive.update()
            telemetry.update()
        }
    }

    private fun preload(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::deposit)

            .addTemporalMarker {
                bot.lift.height = LiftConfig.HIGH

                wristPosFunction = bot.wrist::setToForwardsPos
                armPosFunction = bot.arm::setToForwardsPos
            }

            .splineTo(91.0, 50.0, 90.0)

            .splineTo(DEPOSIT_X + .75, DEPOSIT_Y + .5, DEPOSIT_ANGLE - 3)

            .thenRunPreformed(key = 0)


    private fun deposit(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::intakeCycle)

            .addTemporalMarker(LOWER_OFFSET) {
                bot.lift.height = LiftConfig.HIGH - DEPOSIT_DROP_AMOUNT
            }

            .addTemporalMarker(DEPOSIT_OFFSET) {
                bot.claw.openForDeposit()
            }

            .waitTime(DEPOSIT_DELAY + .05)

            .thenRunPreformed(key = 0)


    private fun intakeCycle(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::intake)

            .addTemporalMarker(RETRACT_OFFSET) {
                bot.claw.openForIntakeNarrow()

                bot.lift.height = liftOffsets[cycleNumber]

                armPosFunction = bot.arm::setToBackwardsPos
                wristPosFunction = bot.wrist::setToBackwardsPos
            }

            .inReverse {
                splineTo(INTAKE_X + 0.325, INTAKE_Y + 2, 0.0)
            }

            .thenRunPreformed(key = 0)


    private fun intake(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::depositCycle)

            .addTemporalMarker(CLAW_CLOSE_OFFSET) {
                bot.claw.close()
            }

            .addTemporalMarker(INTAKE_LIFT_OFFSET - 0.25) {
                bot.lift.height = LiftConfig.HIGH + 200
            }

            .addTemporalMarker(INTAKE_LIFT_OFFSET - 0.125) {
                armPosFunction = bot.arm::setToForwardsPos
                wristPosFunction = bot.wrist::setToForwardsPos
            }

            .waitTime(INTAKE_DELAY + 0.25)

            .thenRunPreformed(key = 0)


    private fun depositCycle(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::deposit)
            .preform(key = 1, ::parkPrep)

            .splineTo(
                DEPOSIT_X + 1.125,
                DEPOSIT_Y + 0.625,
                DEPOSIT_ANGLE + 2
            )

            .thenBranchPreformed(trueKey = 0, elseKey = 1) { cycleNumber++ < MAX_CYCLES }


    private fun parkPrep(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::park)

            .addTemporalMarker(RETRACT_OFFSET) {
                bot.intake.enable()
                bot.claw.openForIntakeNarrow()

                bot.lift.height = liftOffsets[5]

                armPosFunction = bot.arm::setToBackwardsPos
                wristPosFunction = bot.wrist::setToBackwardsPos
            }

            .inReverse {
                splineTo(INTAKE_X, INTAKE_Y, 0.0)
            }

            .addTemporalMarker(CLAW_CLOSE_OFFSET - .03) {
                bot.intake.disable()
                bot.claw.close()
            }

            .thenRunPreformed(key = 0)


    private fun park(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose) {
            turn(1.75)

            addTemporalMarker(0.05) {
                bot.lift.goToZero()
                armPosFunction = bot.arm::setToRestingPos
                wristPosFunction = bot.wrist::setToRestingPos
            }

            when (signalZone) {
                3 -> forward(126.0)
                2 -> forward(70.0)
                else -> forward(1.0)
            }
        }
}
