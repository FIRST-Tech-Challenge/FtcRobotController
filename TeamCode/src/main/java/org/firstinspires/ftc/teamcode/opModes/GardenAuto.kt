package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

abstract class GardenAuto(isRed: Boolean): LinearAuto(isRed) {
    override val autoSequence = listOf(
        State.DRIVE_TO_SCORE,
        State.SHOOT,
        State.DRIVE_TO_GARDEN,
        State.PICKUP_GARDEN,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_DRIVER_FLOWER,
        State.PICKUP_DRIVER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_OTHER_FLOWER,
        State.PICKUP_OTHER_FLOWER,
        State.DRIVE_TO_OTHER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.PARK_RIGHT
    )

}