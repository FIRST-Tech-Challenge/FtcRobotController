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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains a basic example of a Linear "OpMode" class. An OpMode is a 'program' that runs in either
 * the autonomous (auto) or the tele-op period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular tele-op OpMode just executes a basic Mecanum drive for a four wheel robot.
 * It includes all the skeletal structure that all linear OpModes contain.
 */

@TeleOp(name="NU: Mecanum TeleOp", group="Linear OpMode")
public class NUMecanumTeleOp extends LinearOpMode {

    // Declare OpMode members (ex. motors, servos, sensors, ...)
    private ElapsedTime runtime = new ElapsedTime(); // the time the robot has been enabled

    // Drivetrain Motors
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    /*
     * runOpMode() is a method. This can be thought of as the main method of the robot code.
     * This method is called once when the driver presses INIT right before the start of a match
     * Here, the components of the robot (motors, servos, ...) are initialized to the hardware map,
     * and are configured to be ready for the match. Then, when the match starts, the code enters
     * main loop. The code in this loop will run repeatedly for the entirety of the match. In the
     * loop is where the motors are assigned power (in tele-op, an example would be controlling
     * the drivetrain with the game pad). At the end of the match, the robot is stopped
     * and the loop is escaped.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        // This is where we connect (metaphorically) the physical hardware to their Objects in code.
        // Note that the strings used here as parameters to 'get' must correspond to the names
        // assigned during the robot configuration step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Game Pad (the controller) Values

            // Forward/Backward control
            // Up on the Y stick is a negative value so we need to negate it (We want Up to be positive)
            double ly = -gamepad1.left_stick_y;
            // Strafe left/right control
            // We can counteract imperfect strafing with the multiplicand. Test without it to see the difference.
            double lx = gamepad1.left_stick_x * 1.1;
            // Rotation control
            // Right on the X stick (positive value) is for turning clockwise.
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);
            double frontLeftPower = (ly + lx + rx) / denominator;
            double backLeftPower = (ly - lx + rx) / denominator;
            double frontRightPower = (ly - lx - rx) / denominator;
            double backRightPower = (ly + lx - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Controller", "ly: (%.2f), lx: (%.2f), rx: (%.2f)", ly, lx, rx);
            telemetry.addData("Denominator", "denominator: (%.2f)", denominator);
            telemetry.addData("Motors", "frontLeftPower: (%.2f), backLeftPower: (%.2f), frontRightPower: (%.2f), backRightPower: (%.2f)", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
