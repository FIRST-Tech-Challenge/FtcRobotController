package org.firstinspires.ftc.teamcode.drivetrain.messages

class TankCommandMessage(voltage: Double, leftPower: Double, rightPower: Double) {
    var timestamp: Long
    var voltage: Double
    var leftPower: Double
    var rightPower: Double

    init {
        this.timestamp = System.nanoTime()
        this.voltage = voltage
        this.leftPower = leftPower
        this.rightPower = rightPower
    }
}
