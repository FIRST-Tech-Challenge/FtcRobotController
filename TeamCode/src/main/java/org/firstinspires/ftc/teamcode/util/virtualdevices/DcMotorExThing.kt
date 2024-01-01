package org.firstinspires.ftc.teamcode.util.virtualdevices
//
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.hardware.DcMotorController
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.hardware.DcMotorSimple
//import com.qualcomm.robotcore.hardware.HardwareDevice
//import com.qualcomm.robotcore.hardware.PIDCoefficients
//import com.qualcomm.robotcore.hardware.PIDFCoefficients
//import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
//
///** The first motor will be the primary motor, and will be used to find most member information. */
//class DcMotorExThing(public var motor1: DcMotorEx, public var motor2: DcMotorEx) : DcMotorEx {
//    var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
//    var power: Double = 0.0
//    lateinit var motorType: MotorConfigurationType
//    var speed: Double = 0.0
//    lateinit var mode: DcMotor.RunMode
//    var enabled: Boolean = true
//    var position: Int = 0;
//    lateinit var zeroPowerBehavior: DcMotor.ZeroPowerBehavior;
//    init {
//
//    }
//    override fun getManufacturer(): HardwareDevice.Manufacturer { return motor1.manufacturer}
//    override fun getDeviceName(): String { return motor1.deviceName }
//    override fun getConnectionInfo(): String {
//        return motor1.connectionInfo + "\n" + motor2.connectionInfo
//    }
//    override fun getVersion(): Int { return motor1.version }
//    override fun resetDeviceConfigurationForOpMode() {
//        motor1.resetDeviceConfigurationForOpMode()
//        motor2.resetDeviceConfigurationForOpMode()
//    }
//    override fun close() {
//        motor1.close()
//        motor2.close()
//    }
//    override fun setDirection(direction: DcMotorSimple.Direction) { this.direction = direction }
//    override fun getDirection(): DcMotorSimple.Direction = this.direction
//    override fun setPower(power: Double): Unit {
//        motor1.power = power
//        motor2.power = power
//        this.power = power
//    }
//    override fun getPower(): Double = this.power
//    override fun getMotorType(): MotorConfigurationType = this.motorType
//    override fun setMotorType(motorType: MotorConfigurationType?): Unit {
//        if (motorType != null) {
//            this.motorType = motorType
//        }
//    }
//    override fun getController(): DcMotorController = motor1.controller
//    override fun getPortNumber(): Int = motor1.portNumber
//    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
//        motor1.zeroPowerBehavior = zeroPowerBehavior;
//        motor2.zeroPowerBehavior = zeroPowerBehavior;
//    }
//    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior {
//        TODO("Not yet implemented")
//    }
//    override fun setPowerFloat() {
//        TODO("Not yet implemented")
//    }
//    override fun getPowerFloat(): Boolean {
//        TODO("Not yet implemented")
//    }
//    override fun setTargetPosition(position: Int) {
//        motor1.targetPosition = position;
//        motor2.targetPosition = position;
//        this.targetPosition
//    }
//    override fun getTargetPosition(): Int = this.targetPosition
//    override fun isBusy(): Boolean { return motor1.isBusy || motor2.isBusy }
//    override fun getCurrentPosition(): Int = motor1.currentPosition
//    override fun setMode(mode: DcMotor.RunMode) { this.mode = mode; }
//    override fun getMode(): DcMotor.RunMode = this.mode
//    override fun setMotorEnable() { this.enabled = true; }
//    override fun setMotorDisable() { this.enabled = false }
//    override fun isMotorEnabled(): Boolean = this.enabled
//    override fun setVelocity(angularRate: Double) {
//        motor1.setVelocity(angularRate)
//        motor2.setVelocity(angularRate)
//        this.speed = angularRate
//    }
//    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
//        motor1.setVelocity(angularRate, unit)
//        motor2.setVelocity(angularRate, unit)
//        this.speed = angularRate
//    }
//    override fun getVelocity(): Double = this.speed
//    override fun getVelocity(unit: AngleUnit?): Double = this.speed
//    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) {
//        TODO("Not yet implemented")
//    }
//    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) {
//        TODO("Not yet implemented")
//    }
//    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
//        TODO("Not yet implemented")
//    }
//    override fun setPositionPIDFCoefficients(p: Double) {
//        TODO("Not yet implemented")
//    }
//    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients {
//        TODO("Not yet implemented")
//    }
//    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients {
//        TODO("Not yet implemented")
//    }
//    override fun setTargetPositionTolerance(tolerance: Int) {
//        TODO("Not yet implemented")
//    }
//    override fun getTargetPositionTolerance(): Int {
//        TODO("Not yet implemented")
//    }
//    override fun getCurrent(unit: CurrentUnit?): Double {
//        TODO("Not yet implemented")
//    }
//    override fun getCurrentAlert(unit: CurrentUnit?): Double {
//        TODO("Not yet implemented")
//    }
//    override fun setCurrentAlert(current: Double, unit: CurrentUnit?) {
//        TODO("Not yet implemented")
//    }
//    override fun isOverCurrent(): Boolean {
//        TODO("Not yet implemented")
//    }
//}