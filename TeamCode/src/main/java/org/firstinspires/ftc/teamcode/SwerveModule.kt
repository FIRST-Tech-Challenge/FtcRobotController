package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor

class SwerveModule (topMotor: DcMotor, bottomMotor: DcMotor, x: Double, y: Double) {
    var top: DcMotor = topMotor
    var bottom: DcMotor = bottomMotor
}