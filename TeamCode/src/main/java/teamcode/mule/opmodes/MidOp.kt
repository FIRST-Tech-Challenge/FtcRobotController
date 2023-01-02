package teamcode.mule.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
open class MidOp : OpMode() {

    lateinit var arm: DcMotor

    private fun armControl() {
        if(gamepad1.left_bumper) {
            arm.power = 1.0
        }
        else if (gamepad1.right_bumper) {
            arm.power = -1.0
        }
        else {
            arm.power = 0.0
        }
    }

    override fun init() {
        arm = hardwareMap.get(DcMotor::class.java, "Arm")
    }

    override fun loop() {
        armControl()
        telemetry.addData("arm power", arm.power)
    }
}