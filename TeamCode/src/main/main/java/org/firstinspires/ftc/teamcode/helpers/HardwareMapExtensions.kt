package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.helpers.vision.ViewId

fun HardwareMap.getMotor(name: String): DcMotorEx = this.get(DcMotorEx::class.java, name)
fun HardwareMap.getServo(name: String): Servo = this.get(ServoImplEx::class.java, name)
fun HardwareMap.leftFrontMotor(): LeftFrontMotor = this.getMotor("left_front_motor")
fun HardwareMap.rightFrontMotor(): RightFrontMotor = this.getMotor("right_front_motor")
fun HardwareMap.leftBackMotor(): LeftBackMotor = this.getMotor("left_back_motor")
fun HardwareMap.rightBackMotor(): RightBackMotor = this.getMotor("right_back_motor")
fun HardwareMap.mecanumDrive(): MecanumDrive = MecanumDrive(
    this.leftFrontMotor(),
    this.rightFrontMotor(),
    this.leftBackMotor(),
    this.rightBackMotor()
)

fun HardwareMap.webcam(): WebcamName = this.get(WebcamName::class.java, "webcam_1")
fun HardwareMap.cameraMonitorViewId(): ViewId = this.appContext.resources.getIdentifier(
    "cameraMonitorViewId",
    "id",
    this.appContext.packageName
)

var HardwareMap.bulkCachingMode: BulkCachingMode
    get() = TODO()
    set(value) {
        for (module in getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(value)
        }
    }