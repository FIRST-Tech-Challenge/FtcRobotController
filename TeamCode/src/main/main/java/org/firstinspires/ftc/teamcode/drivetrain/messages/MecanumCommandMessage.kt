package org.firstinspires.ftc.teamcode.drivetrain.messages

class MecanumCommandMessage(
    voltage: Double,
    leftFrontPower: Double,
    leftBackPower: Double,
    rightBackPower: Double,
    rightFrontPower: Double
) {
    var timestamp: Long
    var voltage: Double
    var leftFrontPower: Double
    var leftBackPower: Double
    var rightBackPower: Double
    var rightFrontPower: Double

    init {
        this.timestamp = System.nanoTime()
        this.voltage = voltage
        this.leftFrontPower = leftFrontPower
        this.leftBackPower = leftBackPower
        this.rightBackPower = rightBackPower
        this.rightFrontPower = rightFrontPower
    }
}
