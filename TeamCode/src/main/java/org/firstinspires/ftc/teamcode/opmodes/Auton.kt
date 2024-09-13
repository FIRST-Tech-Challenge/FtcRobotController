package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.geometry.Pose2d
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@Config
object AutonConfig {
    @JvmField
    var multiF = 0.1

    @JvmField
    var multiH = 0.1
}

@Config
@TeleOp(name = "Auton")
class Auton : CommandOpMode() {
    private var drive: DriveSubsystem? = null
    var tel: Telemetry? = null
    var pos: Pose2d = Pose2d()
    var dash: FtcDashboard? = null

    val path = listOf(

        Vec2d(-31.333333333333332, 57.333333333333336),
        Vec2d(-39.24444444444445, 50.13333333333333),
        Vec2d(-53.06666666666667, 42.15555555555556),
        Vec2d(-55.06666666666667, 35.733333333333334),
        Vec2d(-57.06666666666667, 29.311111111111114),
        Vec2d(-43.04444444444445, 26.200000000000003),
        Vec2d(-43.333333333333336, 18.8),
        Vec2d(-43.62222222222222, 11.399999999999999),
        Vec2d(-55.37777777777777, 0.2666666666666675),
        Vec2d(-56.8, -8.666666666666666),
        Vec2d(-58.22222222222222, -17.6),
        Vec2d(-53.51111111111111, -26.08888888888889),
        Vec2d(-51.86666666666667, -34.8),
    )

    override fun initialize() {
        val drive = DriveSubsystem(hardwareMap)
        this.drive = drive
        tel = Telemetry()
        dash = FtcDashboard.getInstance()

        schedule(PurePursuitCommand(drive, path, dash!!))
    }

    override fun run() {
        drive!!.updatePos()
        super.run()
    }
}