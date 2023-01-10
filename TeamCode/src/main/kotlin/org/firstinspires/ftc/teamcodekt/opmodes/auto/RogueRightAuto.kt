package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcode.AutoData

@Autonomous
class RogueRightAuto : RogueBaseAuto() {
    override fun executeOrder66() {
        val startPose = GlobalUnits.pos(91, -159, 90)
        val startTraj = mainTrajRefactor(startPose)

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        Scheduler.launchOnStart(opmode = this) {
            bot.updateBaseComponents()
            bot.drive.update()
            mTelemetry.update()
        }
    }

    private fun mainTrajRefactor(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose)
            .initialDepositPrep()

            .waitInitialGoToDeposit()

            .awaitDeposit()

            .doTimes(5) {
                when (it) {
                    4 -> fastIntakePrep()
                    else -> regularIntakePrep()
                }

                awaitGoToIntake(it)

                when (it) {
                    4 -> awaitFastIntake()
                    else -> awaitRegularIntake()
                }

                awaitGoToDeposit(it)

                awaitDeposit()
            }

            .resetBot()


    private fun Anvil.initialDepositPrep() = this
        .addTemporalMarker {
            bot.lift.goToHigh()
            bot.claw.close()
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }


    private fun Anvil.waitInitialGoToDeposit() = this
        .splineToSplineHeading(82.35, -11.35, 129.5, 117.5)


    private fun Anvil.awaitGoToDeposit(it: Int) = this
        .splineToSplineHeading(82.35 + (it * .35), -11.35 - (it * 3.7), 131.5 + (it * 1.9), 155.0)


    private fun Anvil.awaitGoToIntake(it: Int) =
        inReverse {
            splineTo(159.5 - ((it * .6) pow 1.25), -28.75 - (it * .85), 0.0)
        }


    private fun Anvil.awaitDeposit() = this
        .addTemporalMarker(-65) {
            bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
        }

        .addTemporalMarker {
            bot.claw.openForDeposit()
        }

        .waitTime(100)


    private fun Anvil.regularIntakePrep() = this
        .addTemporalMarker {
            bot.lift.height = 250

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeWide()
        }


    private fun Anvil.fastIntakePrep() = this
        .addTemporalMarker {
            bot.lift.height = 250

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }


    private fun Anvil.awaitRegularIntake() = this
        .addTemporalMarker(-75) {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(200) {
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
            bot.lift.goToHigh()
        }

        .waitTime(225)


    private fun Anvil.awaitFastIntake() = this
        .addTemporalMarker(-75) {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(35) {
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
            bot.lift.goToHigh()
        }

        .waitTime(20)


    private fun Anvil.resetBot() = this
        .addTemporalMarker {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            bot.lift.goToZero()
        }
        .waitTime(1000)


    private fun mainTraj(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .addTemporalMarker {
                bot.lift.goToHigh()
                bot.claw.close()
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

            .splineToSplineHeading(82.35, -11.35, 129.5, 117.5) // Preload

            .doTimes(5) {
                addTemporalMarker(-65) {
                    bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
                }

                addTemporalMarker {
                    bot.claw.openForDeposit()
                }

                waitTime(110)

                addTemporalMarker {
                    bot.lift.height = 250

                    bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                    bot.wrist.setToBackwardsPos()

                    if (it < 4) {
                        bot.claw.openForIntakeWide()
                    } else {
                        bot.claw.openForIntakeNarrow()
                        bot.intake.enable()
                    }
                }

                inReverse {
                    splineTo(159.5 - ((it * .6) pow 1.25), -28.75 - (it * .85), 0.0) // Intake
                }

                addTemporalMarker(-75) {
                    bot.intake.disable()
                    bot.claw.openForIntakeNarrow()
                }

                addTemporalMarker(if (it < 4) 200 else 35) {
                    bot.arm.setToForwardsPos()
                    bot.wrist.setToForwardsPos()
                    bot.lift.goToHigh()
                }

                waitTime(if (it < 4) 225 else 20)

                splineToSplineHeading(82.35 + (it * .35), -11.35 - (it * 3.7), 131.5 + (it * 1.9), 155.0) // Deposit
            }

            .addTemporalMarker {
                bot.arm.setToRestingPos()
                bot.wrist.setToRestingPos()
                bot.lift.goToZero()
            }
            .waitTime(1000)
}
