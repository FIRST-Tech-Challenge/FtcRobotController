package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.END_IN;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.END_OUT;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.END_STOP;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.MIDDLE_IN;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.MIDDLE_OUT;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Rollers.MIDDLE_STOP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Feed rollers: intake (front roller), middle, end.
 *
 * PHASE 1: intake, middle, and end are handled safely. If any motor is null
 * (missing from robot configuration), its methods do nothing so it never crashes.
 */
public class Rollers {

	public final DcMotor intake;
	public final DcMotor middle;
	public final DcMotor end;

	public Rollers(DcMotor intake, DcMotor middle, DcMotor end) {
		this.intake = intake;
		this.middle = middle;
		this.end = end;

		if (intake != null) {
			intake.setDirection(DcMotorSimple.Direction.REVERSE);
			intake.setPower(INTAKE_STOP);
		}

		if (middle != null) {
			middle.setPower(MIDDLE_STOP);
		}

		if (end != null) {
			end.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			end.setPower(END_STOP);
		}
	}

	public boolean hasIntake() {
		return intake != null;
	}

	public boolean hasMiddle() {
		return middle != null;
	}

	public boolean hasEnd() {
		return end != null;
	}

	// ---------- Intake ----------

	/**
	 * @param power 1 = full in, -1 = full out, 0 = stop. Scaled by INTAKE_SPEED.
	 */
	public void setIntakePower(double power) {
		if (intake != null) {
			intake.setPower(INTAKE_SPEED * power);
		}
	}

	public void setintakePower(double power) {
		setIntakePower(power);
	}

	public void intakeIn() {
		setIntakePower(1);
	}

	public void intakeStop() {
		setIntakePower(0);
	}

	public void intakeOut() {
		setIntakePower(-1);
	}

	// ---------- Middle ----------

	public void middleIn() {
		if (middle != null) {
			middle.setPower(MIDDLE_IN);
		}
	}

	public void middleStop() {
		if (middle != null) {
			middle.setPower(MIDDLE_STOP);
		}
	}

	public void middleOut() {
		if (middle != null) {
			middle.setPower(MIDDLE_OUT);
		}
	}

	public double getMiddlePower() {
		return middle != null ? middle.getPower() : 0;
	}

	// ---------- End ----------

	public void endIn() {
		if (end != null) {
			end.setPower(END_IN);
		}
	}

	public void endStop() {
		if (end != null) {
			end.setPower(END_STOP);
		}
	}

	public void endOut() {
		if (end != null) {
			end.setPower(END_OUT);
		}
	}

	public double getEndPower() {
		return end != null ? end.getPower() : 0;
	}
}