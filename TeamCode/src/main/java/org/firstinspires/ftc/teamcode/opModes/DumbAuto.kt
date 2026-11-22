package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

abstract class DumbAuto(isRed: Boolean): LinearAuto(isRed) {

    override val autoSequence = listOf(
        State.WAIT_FOR_LEFT_TIP,
        State.DRIVE_TO_SCORE,
        State.SHOOT,
        State.PARK_RIGHT
    )

    override var startPose = poseSet.CORNER_START
}