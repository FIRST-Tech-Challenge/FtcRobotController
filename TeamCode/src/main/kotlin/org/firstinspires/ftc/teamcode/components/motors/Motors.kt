package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.initializableOnce

class Motors {
    var frontLeft: DcMotorEx by initializableOnce()
    var frontRight: DcMotorEx by initializableOnce()
    var backLeft: DcMotorEx by initializableOnce()
    var backRight: DcMotorEx by initializableOnce()
    var intake: DcMotorEx by initializableOnce()
    var lift: DcMotorEx by initializableOnce()
    var duck: DcMotorEx by initializableOnce()
}