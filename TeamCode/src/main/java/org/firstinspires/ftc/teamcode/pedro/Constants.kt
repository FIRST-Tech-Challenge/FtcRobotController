package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.algorithm.Foresight
import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.config.Configuration
import com.pedropathing.controllers.Controller
import com.pedropathing.follower.Follower
import com.pedropathing.math.Matrix
import com.pedropathing.math.Vector2D
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit


object Constants {
    @JvmStatic
    fun create(h: HardwareMap): Follower {
        return Follower(newLocalizer(h), newDrivetrain(h), newForesight())
    }

    var drivetrainConfig: MecanumConfig = MecanumConfig(Configuration { c: MecanumConfig? ->
        c!!.frontLeftName.set("port2")
        c.frontRightName.set("port1")
        c.backLeftName.set("port3")
        c.backRightName.set("port0")
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD)
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE)
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD)
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE)
    })

    var localizerConfig: PinpointConfig = PinpointConfig(Configuration { c: PinpointConfig? ->
        c!!.name.set("odo")
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        c.xPodOffset.set(-7.0)
        c.yPodOffset.set(6.5)
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        c.globalDistanceUnit.set(DistanceUnit.INCH)
        c.offsetUnits.set(DistanceUnit.INCH)
    })

    var foresightConfig: ForesightConfig = ForesightConfig(
        Configuration { c: ForesightConfig? ->
            val primaryTranslationalForward = Controller.proportional(0.2568972329196748)
            val secondaryTranslationalForward = Controller.proportional(0.09491666314333237)
            val primaryTranslationalLateral = Controller.proportional(0.33141797990809874)
            val secondaryTranslationalLateral = Controller.proportional(0.1224500879245225)

            c!!.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                    .put(2.5, primaryTranslationalForward)
            )
            c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                    .put(2.5, primaryTranslationalLateral)
            )

            c.coast.set(Controller.proportionalFeedforward(0.02028701283076027))
            c.brake.set(Controller.proportionalFeedforward(0.01724396090614623))

            c.headingFeedback.set(Controller.proportional(2.6204420331818055))
            c.headingBrakeCoefficients.set(
                Vector2D.cartesian(
                    0.032507246489180255,
                    0.014917238139437527
                )
            )

            c.linearBrakeCoefficients.set(Matrix.diag(0.03978624919415515, 0.012549843364495822))
            c.quadraticBrakeCoefficients.set(
                Matrix.diag(
                    0.0030391409197645506,
                    0.004028811023955176
                )
            )

            c.maxAchievableForwardVelocity.set(51.526981803468075)
            c.maxAchievableStrafeVelocity.set(46.38879405290293)
            c.naturalForwardDeceleration.set(50.32116921646626)
            c.naturalStrafeDeceleration.set(65.69426386210297)
        }
    )

    fun newDrivetrain(hardwaremap: HardwareMap) = Mecanum(hardwaremap, drivetrainConfig)
    fun newLocalizer(hardwaremap: HardwareMap) = PlottingLocalizer(PinpointLocalizer(hardwaremap, localizerConfig))
    fun newForesight() = Foresight(foresightConfig)
}