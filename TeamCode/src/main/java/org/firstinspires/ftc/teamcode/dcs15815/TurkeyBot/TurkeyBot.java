package org.firstinspires.ftc.teamcode.dcs15815.TurkeyBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurkeyBot {
	HardwareMap hwMap;
	public DcMotor backLeft, frontLeft, frontRight, backRight;
	private final double maxPower = 1;

	TurkeyBot(HardwareMap hm) {
		hwMap = hm;
		backLeft = hwMap.dcMotor.get("back_left_motor");
		frontLeft = hwMap.dcMotor.get("front_left_motor");
		frontRight = hwMap.dcMotor.get("front_right_motor");
		backRight = hwMap.dcMotor.get("back_right_motor");

		setMotorDirection(
			   DcMotorSimple.Direction.REVERSE,
			   DcMotorSimple.Direction.FORWARD,
			   DcMotorSimple.Direction.FORWARD,
			   DcMotorSimple.Direction.FORWARD);
	}

	public double clamp(double min, double value, double max) {
		if (value < min) {
			return min;
		} else if (value > max) {
			return max;
		}
		return value;
	}

	public void setPower(double bl, double fl, double fr, double br) {
		backLeft.setPower(clamp(-1 * maxPower, bl, maxPower));
		frontLeft.setPower(clamp(-1 * maxPower, fl, maxPower));
		frontRight.setPower(clamp(-1 * maxPower, fr, maxPower));
		backRight.setPower(clamp(-1 * maxPower, br, maxPower));
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
		double largest = maxPower;
		largest = Math.max(largest, Math.abs(bl));
		largest = Math.max(largest, Math.abs(fl));
		largest = Math.max(largest, Math.abs(fr));
		largest = Math.max(largest, Math.abs(br));

		setPower(bl / largest, fl / largest, fr / largest, br / largest);
	}


	public void drive(double forward, double rotate) {

		double backLeftPower = forward + rotate;
		double frontLeftPower = forward + rotate;
		double frontRightPower = forward - rotate;
		double backRightPower = forward - rotate;

		setProportionalPower(backLeftPower, frontLeftPower, frontRightPower, backRightPower);
	}

	public void stopDriving() {
		setPower(0);
	}
}