package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.teamcode.hardware.HardwareManager


class CDMecanumOtosDrive(private val hardware: HardwareManager) : CDMecanumDrive(hardware) {
    override var localizer = OtosLocalizer(hardware) as Localizer
}