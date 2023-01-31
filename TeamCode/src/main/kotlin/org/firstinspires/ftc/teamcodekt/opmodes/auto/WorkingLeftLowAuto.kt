package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits

@Autonomous
class WorkingLeftLowAuto: RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -15, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .forward(15)
                    .turn(180)
                    .inReverse {
                        splineTo(-84.5, -36.5, 90)
                        turn(49)
                    }

                    .splineTo(-85.2, -35.8, -49)

                    .inReverse {
                        splineTo(-156, -31, 180)
                    }

                    .forward(10)
                    .turn(-28)
                    .strafeRight(8)
                    .forward(18)

                    .doTimes(4) {
                        back(23)
                        waitTime(300)
                        forward(23)
                    }


}