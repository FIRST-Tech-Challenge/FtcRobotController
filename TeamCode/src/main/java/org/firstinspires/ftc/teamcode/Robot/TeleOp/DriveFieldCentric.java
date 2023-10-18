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

package org.firstinspires.ftc.teamcode.Robot.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Field Centric https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 * Gamepad https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
 * proved here https://matthew-brett.github.io/teaching/rotation_2d.html
 * Then, the translation joystick values need to be counterrotated by the robot heading.
 * The IMU returns heading, however we need to rotate the movement counter to the robot’s rotation,
 * so its negative is taken. The joystick values are a vector, and rotating a vector in 2D requires
 * this formula (proved here), where x1 and y1 are the components of the original vector,
 * b is the angle to rotate by, x2 and y2 are the components of the resultant vector.
 *  x2 = x1 * cos(b) - y1 * sin(b)
 *  y2 = x1 * sin(b) + y1 * cos(b)
 */

@TeleOp(name="Drive: Field Centric", group="Robot")
//@Disabled
public class DriveFieldCentric extends LinearOpMode {

    /* Declare OpMode members. */
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };
    double motorPower[] = new double[]{0.0, 0.0, 0.0, 0.0};
    double botHeading = 0.0;

    IMU imu;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    @Override
    public void runOpMode() {

        Gamepad gamepad1Current = new Gamepad();
        Gamepad gamepad1Previous = new Gamepad();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match that of the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        for (int i = 0; i < motor.length; i++)
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motor[0].setDirection(DcMotor.Direction.REVERSE);
        motor[1].setDirection(DcMotor.Direction.REVERSE);
        motor[2].setDirection(DcMotor.Direction.FORWARD);
        motor[3].setDirection(DcMotor.Direction.FORWARD);

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // gamepad1previous to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            gamepad1Previous.copy(gamepad1Current);
            gamepad1Current.copy(gamepad1);

            double y = -gamepad1Current.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1Current.left_stick_x;
            double rx = gamepad1Current.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1Current.options) {
                imu.resetYaw();
            }

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            motorPower[0] = (rotY + rotX + rx) / denominator;
            motorPower[1] = (rotY - rotX + rx) / denominator;
            motorPower[2] = (rotY - rotX - rx) / denominator;
            motorPower[3] = (rotY + rotX - rx) / denominator;

            for (int i = 0; i < motor.length; i++)
                motor[i].setPower(motorPower[i]);

            // Send telemetry message to signify robot running;

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
