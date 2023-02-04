package org.firstinspires.ftc.teamcodekt.opmodes.deprecated

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto

@Disabled
@Deprecated("Never fully tested on comp field, plus mid is just better")
@Autonomous
class RogueLeftLowAuto: RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -159, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .forward(15)
                    .initialDepositPrep()
                    .turn(180)
                    .inReverse {
                        splineTo(-84.5, -36.5, 90)
                        turn(49)
                    }

                    .splineTo(-85.2, -35.8, -49)
                    .deposit()

                    .waitTime(100)

                    .addTemporalMarker(0) {
                        bot.lift.targetHeight = liftOffsets[0]
                    }

                    .inReverse {
                        splineTo(-156, -31, 180)
                    }

                    .forward(10)
                    .turn(-28)
                    .strafeRight(8)
                    .forward(18)

                    .doTimes(4) {
                        regularIntakePrep(it+1)
                        back(23)
                        intake()
                        waitTime(300)
                        depositPrep()
                        forward(23)
                        deposit()
                    }

    private fun Anvil.initialDepositPrep() = this
            .addTemporalMarker(400) {
                bot.lift.goToHigh() // TODO: IN THE FUTURE, CHANGE THIS TO MID - ONLY HAVE A HIGH POLE AT MY HOUSE
                bot.claw.close()
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

    private fun Anvil.depositPrep() = this
            .addTemporalMarker(400) {
                bot.claw.close()
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

    private fun Anvil.deposit() = this
            .addTemporalMarker(-165) {
                bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
            }

            .addTemporalMarker(-100) {
                bot.claw.openForDeposit()
            }

    private fun Anvil.regularIntakePrep(iterations: Int) = this
            .addTemporalMarker(185) {
                bot.lift.targetHeight = liftOffsets[iterations]
                bot.wrist.setToBackwardsPos()
                bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            }

            .addTemporalMarker(325) {
                bot.claw.openForIntakeWide()
            }

    private fun Anvil.intake() = this
            .addTemporalMarker {
                bot.intake.disable()
                bot.claw.close()
            }

            .addTemporalMarker(275) {
                bot.lift.goToLow()
            }

            .addTemporalMarker(425) {
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

            .waitTime(300)


}