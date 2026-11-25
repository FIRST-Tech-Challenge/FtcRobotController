package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

abstract class FusionAuto (isRed: Boolean): LinearAuto(isRed){

    override val autoSequence = listOf (
        State.WAIT_UNDER_HIVE,
        State.WAIT_FOR_TIME,
        State.DRIVE_TO_LEFT_SCORE,
        State.SHOOT,
        State.PARK_LEFT
    )

    override var startPose = poseSet.CORNER_START
}