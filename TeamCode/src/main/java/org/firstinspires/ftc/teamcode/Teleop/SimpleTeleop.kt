package org.firstinspires.ftc.teamcode.Teleop

//import org.firstinspires.ftc.teamcode.Variables.rMotorL
//import org.firstinspires.ftc.teamcode.Variables.rMotorR
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR

@TeleOp(name = "SimpleTeleop", group = "TeleopFinal")
class SimpleTeleop: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondSimple() //init rack and pinion & wheel motors

        telemetry.update()


        waitForStart()
        var leftY: Double
        var leftX: Double
        var rightX: Double
        var upOrDown = true
        var speedDiv = 3.0
        while (opModeIsActive()) {
            //set gamepad inputs
            leftY = (-gamepad1.left_stick_y).toDouble()
            leftX = -gamepad1.left_stick_x.toDouble()
            rightX = -gamepad1.right_stick_x.toDouble()

            //set motor speeds
            val motorFLPower = (leftY - leftX - rightX) / speedDiv
            val motorBLPower = (leftY + leftX - rightX) / speedDiv
            val motorFRPower = (leftY + leftX + rightX) / speedDiv
            val motorBRPower = (leftY - leftX + rightX) / speedDiv

            telemetry.addLine("Power of motorFL: $motorFLPower")
            telemetry.addLine("Power of motorFR: $motorFRPower")
            telemetry.addLine("Power of motorBL: $motorBLPower")
            telemetry.addLine("Power of motorBR: $motorBRPower")

            setMotorPowers(motorFLPower, motorFRPower, motorBLPower, motorBRPower)

            
            if (gamepad1.left_bumper) {
                motorFL?.power = 1.0/speedDiv
                motorBL?.power = -1.0/speedDiv
                motorFR?.power = 1.0/speedDiv
                motorBR?.power = -1.0/speedDiv
            }

            if (gamepad1.right_bumper) {
                motorFL?.power = -1.0/speedDiv
                motorBL?.power = 1.0/speedDiv
                motorFR?.power = -1.0/speedDiv
                motorBR?.power = 1.0/speedDiv
            }

            if (gamepad1.a) {
                if (speedDiv == 3.0) {
                    speedDiv = 1.5
                } else {
                    speedDiv = 3.0
                }
            }

            telemetry.update()

        }
    }
}
