package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.bsmPose2d
import ftc.rogue.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcode.AutoData

@Autonomous
class AnvilRightTestAuto : RogueBaseAuto() {
    override fun executeOrder66() {
        val startPose = bsmPose2d(91, -159, 90)
        val startTraj = mainTraj(startPose)

        waitForStart()

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        Scheduler.launchOnStart(opmode = this) {
            bot.updateComponents()
            bot.drive.update()
            mTelemetry.update()
        }
    }

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

                addTemporalMarker( if (it < 4) 200 else 35 ) {
                    bot.arm.setToForwardsPos()
                    bot.wrist.setToForwardsPos()
                    bot.lift.goToHigh()
                }

                waitTime( if (it < 4) 225 else 20 )

                splineToSplineHeading(82.35 + (it * .35), -11.35 - (it * 3.7), 131.5 + (it * 1.9), 155.0) // Deposit
            }

            .addTemporalMarker {
                bot.arm.setToRestingPos()
                bot.wrist.setToRestingPos()
                bot.lift.goToZero()
            }
            .waitTime(1000)

    private fun mainTrajRefactor(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .addTemporalMarker {
                bot.lift.goToHigh()
                bot.claw.close()
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

            .splineToSplineHeading(77.35, -9.35, 129.5, 117.5)

            .awaitDeposit()

            .doTimes(5) {
                when (it) {
                    4 -> fastIntakePrep()
                    else -> regularIntakePrep(it)
                }

                awaitGoToIntake(it)

                when (it) {
                    4 -> awaitFastIntake()
                    else -> awaitRegularIntake()
                }

                awaitGoToDeposit(it)

                awaitDeposit()
            }


    private fun Anvil.awaitGoToDeposit(iteration: Int) =
        splineToSplineHeading(77.35, -9.35 - ((iteration * .7) pow 2), 129.5, 140.0)


    private fun Anvil.awaitGoToIntake(iteration: Int) =
        inReverse {
            splineTo(153.5, -27.5 - ((iteration * .7) pow if (iteration < 4) 2.5 else 2.15), 0.0) // Intake
        }


    private fun Anvil.awaitDeposit() = this
        .addTemporalMarker(-65) {
            bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
        }

        .addTemporalMarker {
            bot.claw.openForDeposit()
        }

        .waitTime(100)


    private fun Anvil.regularIntakePrep(iteration: Int) = this
        .addTemporalMarker {
            bot.lift.height = 250

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeWide()
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


    private fun Anvil.fastIntakePrep() = this
        .addTemporalMarker {
            bot.claw.openForIntakeNarrow()
            bot.intake.enable()

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeWide()
        }


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
}
