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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp
public class PIDFControl extends LinearOpMode {

	// Declare OpMode members.
	static double MAX_TICKS_PER_SECOND = 2800.0;

	// Motors
	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

    /*
        // UNCOMMENT IF DOING CUSTOM PID TUNING. F IS FEED-FORWARD WHICH ALLOWS THE MOTOR TO "HOLD POSITION".
        static double frontLeft_kP = 0.3;
        static double frontLeft_kI = 0.0;
        static double frontLeft_kD = 0.0;
        static double frontLeft_kF = 0.01;

        static double frontRight_kP = 0.3;
        static double frontRight_kI = 0.0;
        static double frontRight_kD = 0.0;
        static double frontRight_kF = 0.01;

        static double backLeft_kP = 0.3;
        static double backLeft_kI = 0.0;
        static double backLeft_kD = 0.0;
        static double backLeft_kF = 0.01;

        static double backRight_kP = 0.3;
        static double backRight_kI = 0.0;
        static double backRight_kD = 0.0;
        static double backRight_kF = 0.01;
    */

	@Override
	public void runOpMode() throws InterruptedException {

		initializeRobot();

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).

		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery


		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			double y = -gamepad1.left_stick_y; // Remember, this is reversed!
			double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
			double rx = -gamepad1.right_stick_x ;

			// Denominator is the largest motor power (absolute value) or 1
			// This ensures all the powers maintain the same ratio, but only when
			// at least one is out of the range [-1, 1]
			double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
			double frontLeftPower = (y + x + rx) / denominator;
			double backLeftPower = (y - x + rx) / denominator;
			double frontRightPower = (y - x - rx) / denominator;
			double backRightPower = (y + x - rx) / denominator;

			//set motor powers
			setThrottle(frontLeftPower, frontRightPower, backLeftPower, backRightPower);




		}
	}

	/**
	 * Initialize the mecanum drive with PID.
	 */
	public void initializeRobot() {
		// Get the motors

		frontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft");
		frontRight = hardwareMap.get(DcMotorEx.class,"FrontRight");
		backLeft = hardwareMap.get(DcMotorEx.class,"BackLeft");
		backRight = hardwareMap.get(DcMotorEx.class,"BackRight");

		// Initialize motor direction
		frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
		frontRight.setDirection(DcMotor.Direction.REVERSE) ;
		backLeft.setDirection(DcMotor.Direction.FORWARD) ;
		backRight.setDirection(DcMotor.Direction.REVERSE) ;

		// Stop and reset encoders
		frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
		frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
		backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
		backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
/*
		// Optional if you want to compare initial PIDF coefficients; these are restored to factory-set on power-cycle
		// Get the PIDF coefficients for the RUN_USING_ENCODER RunMode
        PIDFCoefficients frontLeftPidfOrig = frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients frontRightPidfOrig = frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients backLeftPidfOrig = backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients backRightPidfOrig = backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

		// USE THIS TO CUSTOM TUNE YOUR PID COEFFICIENTS DEFINED ABOVE
		// Change coefficients using methods included with DcMotorEx class
		PIDFCoefficients frontLeftPidfNew = new PIDFCoefficients(MecanumDrive.frontLeft_kP, MecanumDrive.frontLeft_kI, MecanumDrive.frontLeft_kD, MecanumDrive.frontLeft_kF);
		frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, frontLeftPidfNew);
		PIDFCoefficients frontRightPidfNew = new PIDFCoefficients(MecanumDrive.frontRight_kP, MecanumDrive.frontRight_kI, MecanumDrive.frontRight_kD, MecanumDrive.frontRight_kF);
		frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, frontRightPidfNew);
		PIDFCoefficients backLeftPidfNew = new PIDFCoefficients(MecanumDrive.backLeft_kP, MecanumDrive.backLeft_kI, MecanumDrive.backLeft_kD, MecanumDrive.backLeft_kF);
		backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, backLeftPidfNew);
		PIDFCoefficients backRightPidfNew = new PIDFCoefficients(MecanumDrive.backRight_kP, MecanumDrive.backRight_kI, MecanumDrive.backRight_kD, MecanumDrive.backRight_kF);
		backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, backRightPidfNew);
*/
	}

	/**
	 * Sets the velocity of each of the four wheels in ticks per second.
	 *
	 * @param frontLeftVelocity ticks per second the front left wheel should turn
	 * @param frontRightVelocity ticks per second the front right wheel should turn
	 * @param backLeftVelocity ticks per second the back left wheel should turn
	 * @param backRightVelocity ticks per second the back right wheel should turn
	 */
	public void setVelocity(double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity) {
		frontLeft.setVelocity(frontLeftVelocity);
		frontRight.setVelocity(frontRightVelocity);
		backLeft.setVelocity(backLeftVelocity);
		backRight.setVelocity(backRightVelocity);
	}

	// CALL THIS TO CONTROL WHEELS INSTEAD OF setPower
	/**
	 * Sets the velocity of each of the four wheels in ticks per second based on a throttle value between -1.0 and 1.0.
	 *
	 * @param frontLeftVelocity throttle value for the front left wheel should turn
	 * @param frontRightVelocity throttle value for the front right wheel should turn
	 * @param backLeftVelocity throttle value for the back left wheel should turn
	 * @param backRightVelocity throttle value for the back right wheel should turn
	 */
	public void setThrottle(double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity) {
		// Constrain throttle values to between -1.0 to 1.0
		if (frontLeftVelocity > 1.0) {

			frontLeftVelocity = 1.0;

		} else if (frontLeftVelocity < 1.0) {

			frontLeftVelocity = -1.0;

		}

		if (frontRightVelocity > 1.0) {

			frontRightVelocity = 1.0;

		} else if (frontRightVelocity < 1.0) {

			frontRightVelocity = -1.0;

		}

		if (backLeftVelocity > 1.0) {

			backLeftVelocity = 1.0;

		} else if (backLeftVelocity < 1.0) {

			backLeftVelocity = -1.0;

		}

		if (backRightVelocity > 1.0) {

			backRightVelocity = 1.0;

		} else if (backRightVelocity < 1.0) {

			backRightVelocity = -1.0;

		}

		// Set velocities
		frontLeft.setVelocity(frontLeftVelocity * PIDFControl.MAX_TICKS_PER_SECOND);
		frontRight.setVelocity(frontRightVelocity * PIDFControl.MAX_TICKS_PER_SECOND);
		backLeft.setVelocity(backLeftVelocity * PIDFControl.MAX_TICKS_PER_SECOND);
		backRight.setVelocity(backRightVelocity * PIDFControl.MAX_TICKS_PER_SECOND);
	}

}
