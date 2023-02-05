package org.firstinspires.ftc.teamcode.Team.BasicRobots

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.util.function.Consumer

/**
 *    This file is the basic framework for a robot the drives on mecanum wheels
 */
open class Mecanum {
    /**
     *    A list of the motor names
     */
    var names = arrayOf("RFMotor", "RBMotor", "LFMotor", "LBMotor")
    /**
     *    A HashMap of motor name to motor list index
     */
    var motors = mutableMapOf<String, DcMotor>()

    /**
     *    Initializes robot using HardwareMap
     *    Sets motor power to zero
     *    Sets motor to run without encoders
     */
    open fun init(map: HardwareMap) {
        for (name in names) {
            motors[name] = map.dcMotor[name]
        }

        mapAll { m: DcMotor ->
            m.power = 0.0
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        mapRight { m: DcMotor -> m.direction = DcMotorSimple.Direction.FORWARD }
        mapLeft { m: DcMotor -> m.direction = DcMotorSimple.Direction.REVERSE }
    }

    /**
     *    Takes a function and applies it to all the motors
     */
    fun mapAll(f: Consumer<DcMotor>) {
        for (motor in motors.values) {
            f.accept(motor)
        }
    }

    /**
     *    Takes a map of motor name to motor power and sets the motor power accordingly
     */
    fun setPower(map: Map<String, Double>) {
        for ((name, power) in map) {
            motors[name]!!.power = power
        }
    }

    /**
     *    Takes a function and applies it to the right motors
     */
    fun mapRight(f: Consumer<DcMotor>) {
        f.accept(motors["RFMotor"]!!)
        f.accept(motors["RBMotor"]!!)
    }

    /**
     *    Takes a function and applies it to the left motors
     */
    fun mapLeft(f: Consumer<DcMotor>) {
        f.accept(motors["LFMotor"]!!)
        f.accept(motors["LBMotor"]!!)
    }

    /**
     *    Takes a function and applies it to the right-front and left-back motors
     */
    fun mapRightDiagonal(f: Consumer<DcMotor>) {
        f.accept(motors["RFMotor"]!!)
        f.accept(motors["LBMotor"]!!)
    }

    /**
     *    Takes a function and applies it to the left-front and right-back motors
     */
    fun mapLeftDiagonal(f: Consumer<DcMotor>) {
        f.accept(motors["LFMotor"]!!)
        f.accept(motors["RBMotor"]!!)
    }

    /**
     *    Moves the robot forward
     */
    fun moveStraight(power: Double) {
        mapAll { m: DcMotor -> m.power = power }
    }

    /**
     *    Moves the robot left
     */
    fun moveLeft(power: Double) {
        mapLeftDiagonal { m: DcMotor -> m.power = -power }
        mapRightDiagonal { m: DcMotor -> m.power = power }
    }

    /**
     *    Moves the robot right
     */
    fun moveRight(power: Double) {
        mapLeftDiagonal { m: DcMotor -> m.power = power }
        mapRightDiagonal { m: DcMotor -> m.power = -power }
    }

    /**
     *    Moves the robot backward
     */
    fun moveBackwards(power: Double) {
        mapAll { m: DcMotor -> m.power = -power }
    }

    /**
     *    Stops the robot by setting the power of every motor to zero
     */
    fun stop() {
        mapAll { m: DcMotor -> m.power = 0.0 }
    }

    /**
     *    Pivots the robot in-place
     */
    fun pivotTurn(power: Double, rightBumper: Boolean, leftBumper: Boolean) {
        if (rightBumper && leftBumper) {
            stop()
        } else if (leftBumper) {
            mapLeft { m: DcMotor -> m.power = power }
            mapRight { m: DcMotor -> m.power = -power }
        } else if (rightBumper) {
            mapLeft { m: DcMotor -> m.power = -power }
            mapRight { m: DcMotor -> m.power = power }
        }
    }

    /**
     *    true  returns 1.0
     *    false returns 0.0
     */
    fun on(trigger: Boolean): Double {
        return if (trigger) 1.0 else 0.0
    }

    /**
     *    Can move the robot along the four axes and diagonals
     */
    fun octoStrafe(power: Double, up: Boolean, down: Boolean, left: Boolean, right: Boolean) {
        if (up) {
            if (right or left) {
                mapLeftDiagonal { m: DcMotor -> m.power = power * on(right) }
                mapRightDiagonal { m: DcMotor -> m.power = power * on(left) }
            } else {
                mapAll { m: DcMotor -> m.power = power }
            }
        } else if (down) {
            if (right or left) {
                mapLeftDiagonal { m: DcMotor -> m.power = -power * on(left) }
                mapRightDiagonal { m: DcMotor -> m.power = -power * on(right) }
            } else {
                mapAll { m: DcMotor -> m.power = -power }
            }
        } else {
            if (right) {
                mapLeftDiagonal { m: DcMotor -> m.power = power }
                mapRightDiagonal { m: DcMotor -> m.power = -power }
            } else if (left) {
                mapLeftDiagonal { m: DcMotor -> m.power = -power }
                mapRightDiagonal { m: DcMotor -> m.power = power }
            }
        }
    }
}