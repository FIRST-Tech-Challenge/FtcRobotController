package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.follower.Follower
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.pedropathing.revhub.localizers.PinpointConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


object Constants {
    val mecanumConfig: MecanumConfig = MecanumConfig { config: MecanumConfig ->
        config.apply {
            frontLeftName.set("left_front")
            backLeftName.set("left_back")
            frontRightName.set("right_front")
            backRightName.set("right_back")

            frontLeftDirection.set(DcMotorSimple.Direction.REVERSE)
            backLeftDirection.set(DcMotorSimple.Direction.REVERSE)
            frontRightDirection.set(DcMotorSimple.Direction.FORWARD)
            backRightDirection.set(DcMotorSimple.Direction.FORWARD)

            manualBrakeMode.set(false)
        }
    }
    val pinpointConfig: PinpointConfig = PinpointConfig { config: PinpointConfig ->
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
    }
    val foresightConfig: ForesightConfig = ForesightConfig {}
}