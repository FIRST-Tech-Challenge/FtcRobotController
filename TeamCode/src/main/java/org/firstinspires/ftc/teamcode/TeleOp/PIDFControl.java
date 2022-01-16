package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDFControl {
	// Motors
	private DcMotorEx frontLeft;
	private DcMotorEx frontRight;
	private DcMotorEx backLeft;
	private DcMotorEx backRight;

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
	/**
	 * Initialize the mecanum drive with PID.
	 */
	public MecanumDrive() {
		// Get the motors
        frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("FrontLeft");
        frontRight = (DcMotorEx)hardwareMap.dcMotor.get("FrontRight");
        backLeft = (DcMotorEx)hardwareMap.dcMotor.get("BackLeft");
        backRight = (DcMotorEx)hardwareMap.dcMotor.get("BackRight");

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

	// CALL THIS TO CONTROL WHEELS INSTEAD OF setPower
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

}
