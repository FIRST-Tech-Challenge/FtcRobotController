package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits

@Autonomous
class RogueLeftLowAuto: RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -159, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .inReverse {
                        splineTo(-84.5, -36.5, 90)
                        turn(44)
                    }

                    .splineTo(-73.2, -47.8, -44)
                    .awaitDeposit()

                    .inReverse {
                        splineTo(-156, -31, 180)
                        awaitIntake1()
                    }

                    .forward(10)
                    .turn(-28.5)
                    .strafeRight(12.5)
                    .forward(18)
                    .awaitDeposit()

                    .doTimes(4) {
                        back(25)
                        awaitIntake2()
                        forward(25)
                        awaitDeposit()
                    }


    private fun Anvil.awaitDeposit() = this
            .waitTime(350)

    // Regular intaking w/ claw open wide
    private fun Anvil.awaitIntake1() = this
            .waitTime(350)

    // Intaking w/ claw narrow & intake wheels spinning
    private fun Anvil.awaitIntake2() = this
            .waitTime(350)

}