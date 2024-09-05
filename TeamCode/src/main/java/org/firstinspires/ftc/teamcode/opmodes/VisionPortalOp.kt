package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.subsystems.Apriltag
import org.firstinspires.ftc.teamcode.common.subsystems.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor

@Config
@TeleOp(name = "Vision")
class VisionPortalOp: CommandOpMode() {
    var visionPortal: VisionPortal? = null
    var aprilTag: Apriltag? = null
    var tel: Telemetry? = null
    override fun initialize() {
        aprilTag = Apriltag()
        visionPortal = VisionPortal(hardwareMap, "camera1", listOf(aprilTag) as List<VisionProcessor>)
        tel = FtcDashboard.getInstance().telemetry
    }

    override fun run() {
        super.run()
        val tags = aprilTag!!.getTags()

        for (tag in tags) {
            val poseData = "${tag.ftcPose.x}, ${tag.ftcPose.y}, ${tag.ftcPose.z}"
            tel!!.addData(tag.id.toString(), poseData)
        }
        tel!!.update()
    }
}