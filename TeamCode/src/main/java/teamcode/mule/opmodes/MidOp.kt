package teamcode.mule.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
open class MidOp : OpMode() {

    lateinit var arm: DcMotor
    lateinit var liftLead: DcMotor
    lateinit var lift2: DcMotor

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

    private fun liftControl() {
        if(gamepad1.a) {
            liftLead.power = 1.0
            lift2.power = -1.0
        }
        else if (gamepad1.b) {
            liftLead.power = -0.5
            lift2.power = 0.5
        }
        else {
            liftLead.power = 0.0
            lift2.power = 0.0
        }
    }

    override fun init() {
        arm = hardwareMap.get(DcMotor::class.java, "Arm")
        liftLead = hardwareMap.get(DcMotor::class.java, "liftLead")
        lift2 = hardwareMap.get(DcMotor::class.java, "lift2")
    }

    override fun loop() {
        armControl()
        liftControl()
    }
}