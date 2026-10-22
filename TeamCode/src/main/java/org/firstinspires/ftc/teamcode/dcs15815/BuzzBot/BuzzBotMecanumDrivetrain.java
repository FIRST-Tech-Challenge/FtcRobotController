package org.firstinspires.ftc.teamcode.dcs15815.BuzzBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot.DefenderBot;
import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot.DefenderBotDrivetrain;


public class BuzzBotMecanumDrivetrain extends DefenderBotDrivetrain {

	public DcMotorEx backLeft, frontLeft, frontRight, backRight;

	public ElapsedTime drivingTimer;
	public double powerBoost = 0;
	public boolean allowPowerBoost = true;

	public BuzzBotMecanumDrivetrain(HardwareMap hm, DefenderBot b) {
		super(hm, b);

//		backLeft = hm.get(DcMotorEx.class, DecodeConfiguration.DRIVETRAIN_MOTOR_BACK_LEFT_NAME);
//		frontLeft = hm.get(DcMotorEx.class, DecodeConfiguration.DRIVETRAIN_MOTOR_FRONT_LEFT_NAME);
//		frontRight = hm.get(DcMotorEx.class, DecodeConfiguration.DRIVETRAIN_MOTOR_FRONT_RIGHT_NAME);
//		backRight = hm.get(DcMotorEx.class, DecodeConfiguration.DRIVETRAIN_MOTOR_BACK_RIGHT_NAME);

		backLeft = (DcMotorEx) hm.dcMotor.get(BuzzBotConfiguration.DRIVETRAIN_MOTOR_BACK_LEFT_NAME);
		frontLeft = (DcMotorEx) hm.dcMotor.get(BuzzBotConfiguration.DRIVETRAIN_MOTOR_FRONT_LEFT_NAME);
		frontRight = (DcMotorEx) hm.dcMotor.get(BuzzBotConfiguration.DRIVETRAIN_MOTOR_FRONT_RIGHT_NAME);
		backRight = (DcMotorEx) hm.dcMotor.get(BuzzBotConfiguration.DRIVETRAIN_MOTOR_BACK_RIGHT_NAME);


		backLeft.setDirection(BuzzBotConfiguration.DRIVETRAIN_MOTOR_BACK_LEFT_DIRECTION);
		frontLeft.setDirection(BuzzBotConfiguration.DRIVETRAIN_MOTOR_FRONT_LEFT_DIRECTION);
		frontRight.setDirection(BuzzBotConfiguration.DRIVETRAIN_MOTOR_FRONT_RIGHT_DIRECTION);
		backRight.setDirection(BuzzBotConfiguration.DRIVETRAIN_MOTOR_BACK_RIGHT_DIRECTION);

//		frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//		frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//		backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//		backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


		resetEncoders();
	}

	public void resetEncoders() {
		backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
	}

	public void setPower(double bl, double fl, double fr, double br) {
		backLeft.setPower(bl);
		frontLeft.setPower(fl);
		frontRight.setPower(fr);
		backRight.setPower(br);
	}

	public void setPower(double p) {
		setPower(p, p, p, p);
	}

	public void setMotorDirection(DcMotorSimple.Direction bl, DcMotorSimple.Direction fl,
	                              DcMotorSimple.Direction fr, DcMotorSimple.Direction br) {
		backLeft.setDirection(bl);
		frontLeft.setDirection(fl);
		frontRight.setDirection(fr);
		backRight.setDirection(br);
	}

	public void setMotorDirection(DcMotorSimple.Direction left, DcMotorSimple.Direction right) {
		setMotorDirection(left, left, right, right);
	}

	private void setProportionalPower(double bl, double fl, double fr, double br) {
		setProportionalPower(bl, fl, fr, br, BuzzBotConfiguration.DRIVETRAIN_POWER_MAX_AUTONOMOUS);
	}

	private void setProportionalPower(double bl, double fl, double fr, double br, double p) {

		// I think the way this is supposed to work is not scaling based on the largest,
		// but instead off of the front left value—which should be he max value

		bl *= p;
		fl *= p;
		fr *= p;
		br *= p;

//		double largest = DecodeConfiguration.DRIVETRAIN_POWER_MAX_AUTONOMOUS;
		double largest = Math.abs(bl);
//		largest = Math.max(largest, Math.abs(bl));
		largest = Math.max(largest, Math.abs(fl));
		largest = Math.max(largest, Math.abs(fr));
		largest = Math.max(largest, Math.abs(br));

		backLeft.setPower(bl / largest);
		frontLeft.setPower(fl / largest);
		frontRight.setPower(fr / largest);
		backRight.setPower(br / largest);
	}

	@Override
	public void stopDriving() {
		setPower(0);
	}

	public void deactivateTimer() {
		drivingTimer = null;
		powerBoost = 0;
	}

	public void driveWithPinpointValues(double x, double y, double h, double p) {

		if (allowPowerBoost && (drivingTimer == null)) {
			drivingTimer = new ElapsedTime();
			powerBoost = 0;
		}

		p += powerBoost;

		x *= p;
		y *= p;
		h *= p;

//		bot.telemetry.addData("x out", x);
//		bot.telemetry.addData("y out", y);
//		bot.telemetry.addData("h out", h);
//		bot.telemetry.addData("p", p);


		double backLeftOutput = x + -y - h;
		double frontLeftOutput = x - -y - h;
		double frontRightOutput = x + -y + h;
		double backRightOutput = x - -y + h;

		double max = Math.max(Math.abs(frontLeftOutput), Math.abs(frontRightOutput));
		max = Math.max(max, Math.abs(backLeftOutput));
		max = Math.max(max, Math.abs(backRightOutput));

		if (allowPowerBoost && (drivingTimer.milliseconds() > 2000 && max < 0.3)) {
			powerBoost += .05;
		}


		if (max > 1.0) {
			backLeftOutput /= max;
			frontLeftOutput /= max;
			frontRightOutput /= max;
			backRightOutput /= max;
		}


//		bot.telemetry.addData("BL", backLeftOutput);
//		bot.telemetry.addData("FL", frontLeftOutput);
//		bot.telemetry.addData("FR", frontRightOutput);
//		bot.telemetry.addData("BR", backRightOutput);
//		bot.telemetry.addData("boost", powerBoost);

		setPower(backLeftOutput, frontLeftOutput, frontRightOutput, backRightOutput);

	}

	public void drive(double arr[]) {
		drive(arr[0], arr[1], arr[2]);
	}

	public void drive(double forward, double strafe, double rotate) {

		double backLeftPower = forward - strafe + rotate;
		double frontLeftPower = forward + strafe + rotate;
		double frontRightPower = forward - strafe - rotate;
		double backRightPower = forward + strafe - rotate;

		setProportionalPower(backLeftPower, frontLeftPower, frontRightPower, backRightPower);
	}

	public void driveNoProportional(double forward, double strafe, double rotate) {
		double backLeftPower = forward - strafe + rotate;
		double frontLeftPower = forward + strafe + rotate;
		double frontRightPower = forward - strafe - rotate;
		double backRightPower = forward + strafe - rotate;
		setPower(backLeftPower, frontLeftPower, frontRightPower, backRightPower);
	}

	public void driveByVelocity(double forward, double strafe, double rotate) {
//	   setMotorDirection(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
//			 DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

		backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		double backLeftPower = forward - strafe + rotate;
		double frontLeftPower = forward + strafe + rotate;
		double frontRightPower = forward - strafe - rotate;
		double backRightPower = forward + strafe - rotate;

		double largest = 1;
		largest = Math.max(largest, Math.abs(backLeftPower));
		largest = Math.max(largest, Math.abs(frontLeftPower));
		largest = Math.max(largest, Math.abs(frontRightPower));
		largest = Math.max(largest, Math.abs(backRightPower));

		double maxTicksSecond = BuzzBotConfiguration.DRIVETRAIN_MAX_TICKS_PER_SECOND;

		backLeft.setVelocity(backLeftPower / largest * maxTicksSecond);
		frontLeft.setVelocity(frontLeftPower / largest * maxTicksSecond);
		frontRight.setVelocity(frontRightPower / largest * maxTicksSecond);
		backRight.setVelocity(backRightPower / largest * maxTicksSecond);
	}

	public boolean frontLeftIsStalled() {
//        return false;
		return frontLeft.getVelocity() == 0;
	}




}