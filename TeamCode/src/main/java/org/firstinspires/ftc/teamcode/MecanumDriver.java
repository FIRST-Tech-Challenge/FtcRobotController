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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Mecanum Driver", group="TeleOp")
public class MecanumDriver extends OpMode
{
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    public final boolean isFieldCentric = true;


    private ElapsedTime runtime = new ElapsedTime();
    private IMU gyro;
    private double wantedHeading = 0.0;
    private final double TURN_POWER = 1.3;
    private final double FORWARD_POWER = 1.0;
    private final double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final double SPEED_MULTIPLIER = 2.3;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.resetYaw();
    }

    @Override
    public void init_loop() {
    }

    public double getAngleImuDegrees() {
        return normalize(
                gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    public static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    public double calculateP(double power, double deadBand, double wantedHeading) {
        double currentHeading = getAngleImuDegrees();
        double headingCorrection = currentHeading - wantedHeading;
        while (headingCorrection > 180) headingCorrection -= 360;
        while (headingCorrection <= -180) headingCorrection += 360;

        if (Math.abs(headingCorrection) > deadBand) {
            telemetry.addData("Dead band activated", "WARNING dead band was activated, heading correction exceeded limit of " + deadBand + " degrees.");
            return 0.0;
        }
        return headingCorrection * power;
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double turn = 0.0;
        double forward = 0.0;
        double strafe = 0.0;
        double currentHeading = getAngleImuDegrees();

        if (gamepad1.right_stick_x != 0) {
            wantedHeading = currentHeading;
            turn = gamepad1.right_stick_x * TURN_POWER;
        } else {
            turn = calculateP(0.02, 60, wantedHeading);
        }
        if (isFieldCentric) {
            forward = ((gamepad1.left_stick_y * Math.cos(currentHeading * (Math.PI / 180))) +
                    (gamepad1.left_stick_x * Math.sin(currentHeading * (Math.PI / 180)))) * SPEED_MULTIPLIER * FORWARD_POWER;
            strafe = ((gamepad1.left_stick_y * Math.sin(currentHeading * (Math.PI / 180))) -
                (gamepad1.left_stick_x * Math.cos(currentHeading * (Math.PI / 180)))) * SPEED_MULTIPLIER * STRAFE_POWER;
        } else {
            forward = gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER;
            strafe = gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER;
        }

        double backLeftPower = Range.clip((forward + strafe - turn) / 3, -1.0, 1.0);
        double backRightPower = Range.clip((forward + strafe + turn) / 3, -1.0, 1.0);
        double frontLeftPower = Range.clip((forward - strafe - turn) / 3, -1.0, 1.0);
        double frontRightPower = Range.clip((forward - strafe + turn) / 3, -1.0, 1.0);

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);

        telemetry.addData("Forward", "Forward: " + forward);
        telemetry.addData("Strafe", "Strafe: " + strafe);
        telemetry.addData("Turn", "Turn: " + turn);
        telemetry.addData("Heading", "Heading: " + currentHeading);
    }

    @Override
    public void stop() {
    }

}
