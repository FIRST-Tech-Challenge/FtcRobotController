package org.firstinspires.ftc.teamcode.drive.robotcore

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.drive.roadrunner.TwoWheelOdometryLocalizer


class ExtThinBot(_hardwareMap: HardwareMap): BaseRobot(_hardwareMap) {

    @JvmField val intakeMotor1 : DcMotorEx = hwInit("intakeMotor1")
    @JvmField val intakeMotor2 : DcMotorEx = hwInit("intakeMotor2")
    @JvmField val webcamServo : Servo = hwInit("webcamServo")

    @JvmField val imuControllerC = IMUController(hardwareMap, id = 'C')
    override val holonomicRR: HolonomicRR = HolonomicRR(imuControllerC, frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                    TwoWheelOdometryLocalizer(intakeMotor2, intakeMotor1, imuControllerC))

    init {
        intakeMotor1.direction = DcMotorSimple.Direction.REVERSE
    }

}