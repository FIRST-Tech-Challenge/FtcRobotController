package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.listeners.after
import ftc.rogue.blacksmith.util.bsmPose2d
import ftc.rogue.blacksmith.util.kt.pow
import ftc.rogue.blacksmith.util.toIn
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

            .splineToSplineHeading(77.35, -9.35, 129.5, 117.5) // Preload

            .doTimes(5) {
                addTemporalMarker(-65) {
                    bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
                }

                addTemporalMarker {
                    bot.claw.openForDeposit()
                }

                waitTime(100)

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
                    splineTo(153.5, -27.5 - ((it * .7) pow if (it < 4) 2.5 else 2.15), 0.0) // Intake

                    addTemporalMarker(-75) {
                        bot.intake.disable()
                        bot.claw.close()
                    }

                    addTemporalMarker( if (it < 4) 200 else 35 ) {
                        bot.arm.setToForwardsPos()
                        bot.wrist.setToForwardsPos()
                        bot.lift.goToHigh()
                    }

                    waitTime( if (it < 4) 225 else 20 )
                }

                splineToSplineHeading(77.35, -9.35 - ((it * .7) pow 2), 129.5, 140.0) // Deposit
            }
}
