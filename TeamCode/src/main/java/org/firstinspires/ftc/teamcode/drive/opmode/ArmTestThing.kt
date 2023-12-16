package org.firstinspires.ftc.teamcode.drive.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry

@TeleOp(name="im so depressed.....")
class ArmTestThing : LinearOpMode() {
    lateinit var telemetries: Telemetry

    lateinit var motor1: DcMotorEx
    lateinit var motor2: DcMotorEx

    override fun runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx::class.java, "motor6")
        motor2 = hardwareMap.get(DcMotorEx::class.java, "motor7")

        motor1.setMotorDisable()
        motor2.setMotorDisable()

        motor1.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor2.mode = DcMotor.RunMode.RUN_TO_POSITION

        motor1.power = 0.5;
        motor2.power = 0.5;

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor1.targetPosition = 0;
                motor2.targetPosition = 0;
            }
            if (gamepad1.b) {
                motor1.targetPosition = 1;
                motor2.targetPosition = 1;
            }
            if (gamepad1.x) {
                motor1.targetPosition = 400;
                motor2. targetPosition = 400;
            }
            if (gamepad1.y) {
                motor1.targetPosition = 5000;
                motor2.targetPosition = 5000;
            }
            if (gamepad1.left_bumper) {
                motor1.power = 0.1
                motor2.power = 0.1
            }
            if ( gamepad1.right_bumper) {
                motor1.power = 0.5
                motor2.power= 0.5
            }
        }
    }

}