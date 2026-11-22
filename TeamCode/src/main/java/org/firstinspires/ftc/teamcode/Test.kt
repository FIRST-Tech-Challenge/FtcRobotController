package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap

@TeleOp
class Test: LinearOpMode(){
    lateinit var hw: GerthaHardwareMap

    override fun runOpMode() {
        hw = GerthaHardwareMap(hardwareMap)

        waitForStart()
        while (opModeIsActive()){

//            telemetry.addData("ticks", hw.pollenTurretEncoder.getCurrentPosition())
//            telemetry.addData("red", hw.color.red())
//            telemetry.addData("blue", hw.color.blue())
//            telemetry.addData("green", hw.color.green())
//            Log.i("distance", hw.color.getDistance(DistanceUnit.CM).toString())
//            telemetry.addData("distance", hw.color.getDistance(DistanceUnit.CM))
            telemetry.update()
        }
    }
}