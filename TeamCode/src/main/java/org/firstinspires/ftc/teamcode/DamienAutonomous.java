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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name="DamienAutonomous", group="Linear Opmode")
@Disabled
public class DamienAutonomous extends DriveMethods {

    // Declare OpMode members.

    DcMotor motorLinearSlide;
    int currentSlidePosClicks;
    private enum slidePositionEnum {
        LOWEST,
        HIGHEST,
    }
    private void moveSlideToPos(double distanceMeters) {
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter) / 1.15);
        motorLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLinearSlide.setTargetPosition((targetPos));
        motorLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPos = motorLinearSlide.getCurrentPosition();
        if(distanceMeters > 0) {
            motorLinearSlide.setPower(-1/6);
        } else {
            motorLinearSlide.setPower(1/6);
        }

        while(targetPos >= currentPos){
            currentPos = Math.abs(motorLinearSlide.getCurrentPosition());
            telemetry.addLine("Current Position: " + motorLinearSlide.getCurrentPosition());
            telemetry.addLine("Target Position: " + targetPos);
            telemetry.update();
        }
        motorLinearSlide.setPower(0);

    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initMotorsRed();

        initMotorsBlue();
        CalibrateIMU();
        motorLinearSlide = hardwareMap.get(DcMotor.class,"motorLS");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addData("Cumulative Z", "" + getCumulativeZ());
            telemetry.addData("Current Z", "" + getCurrentZ());
            telemetry.addData("Error","" + (0 - getCurrentZ()));
            telemetry.update();
        }
    }
}
