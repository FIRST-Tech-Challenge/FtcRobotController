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

package org.firstinspires.ftc.teamcode.To_Be_Removed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Constants;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Deprecated TeleOp", group="Linear OpMode")
@Disabled
public class DeprecatedTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor intake;
    private  DcMotor launcher;
    private Servo wrist;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //Motor Behaviors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotor.Direction.FORWARD);

        //Initial Positions
        wrist.setPosition(Constants.initialWristPosition);

        //Initialize PID
        //MiniPID armPID = new MiniPID(1, 0, 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive Code
            // Setup a variable for each drive wheel to save power level for telemetry
            double backLeftPower;
            double backRightPower;
            double frontLeftPower;
            double frontRightPower;

            double forwardDrive = -gamepad1.left_stick_y;
            double shuffleDrive  =  gamepad1.left_stick_x;
            double turnDrive = gamepad1.right_stick_x;

            frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
            backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);

            // Send calculated power to wheels
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);

            //Intake Power
            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            //Launcher Power
            if (gamepad2.x) {
                launcher.setPower(Constants.launcherPower);
            }
            else {
                launcher.setPower(0.0);
            }

            //Wrist Code
            if (gamepad2.right_bumper) {
                wrist.setPosition(Range.clip(wrist.getPosition() + Constants.wristPositionIncrement, 0.0, 1.0));
            }

            else if (gamepad2.left_bumper) {
                wrist.setPosition(Range.clip(wrist.getPosition() - Constants.wristPositionIncrement, 0.0, 1.0));
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Motors", "frontLeft (%.2f), backLeft (%.2f), frontRight (%.2f), backRight (%.2f)", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Intake Motor", intake.getPower());
            telemetry.addData("Launcher", launcher.getPower());
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();
        }
    }
}
