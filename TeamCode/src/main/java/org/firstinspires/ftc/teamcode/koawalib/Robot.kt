package org.firstinspires.ftc.teamcode.koawalib

import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Slides

class Robot() {
    val hardware = Hardware()

    val drive = KMecanumDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr
    )

    val armServo = Arm(hardware.arm)
    val clawServo = Claw(hardware.claw)
    val slidesMotor = Slides(hardware.slides)


}