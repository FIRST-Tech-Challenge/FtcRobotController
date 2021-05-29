/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.driver

import android.content.Context
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.R
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement

class EncoderDrive(val robot: Robot){

    private val runtime = ElapsedTime()

    val COUNTS_PER_MOTOR_REV = 1440.0

    val DRIVE_GEAR_REDUCTION = 2.0

    val WHEEL_DIAMETER_INCHES = 4.0

    val COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
            (WHEEL_DIAMETER_INCHES * 3.1415)
    val DRIVE_SPEED = 0.6
    val TURN_SPEED = 0.5

    fun encoderDrive(
            speed: Double,
            leftInches: Double,
            rightInches: Double,
    ) {
        val newLeftTarget: Int
        val newRightTarget: Int

        val leftDrive = (robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.leftDriveID)) as DcMotor)
        val rightDrive = (robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.leftDriveID)) as DcMotor)


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.currentPosition + (leftInches * COUNTS_PER_INCH).toInt()
        newRightTarget = rightDrive.currentPosition + (rightInches * COUNTS_PER_INCH).toInt()
        leftDrive.targetPosition = newLeftTarget
        rightDrive.targetPosition = newRightTarget

        // Turn On RUN_TO_POSITION
        leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        // reset the timeout time and start motion.
        runtime.reset()
        leftDrive.power = Math.abs(speed)
        rightDrive.power = Math.abs(speed)

        // Stop all motion;
        leftDrive.power = 0.0
        rightDrive.power = 0.0

        // Turn off RUN_TO_POSITION
        leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}