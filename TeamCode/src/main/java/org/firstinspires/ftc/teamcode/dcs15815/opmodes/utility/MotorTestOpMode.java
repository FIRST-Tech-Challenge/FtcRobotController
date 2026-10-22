package org.firstinspires.ftc.teamcode.dcs15815.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Utility(name = "Motor Test", description="")
public class MotorTestOpMode extends LinearOpMode {
	DcMotor motor;

	@Override
	public void runOpMode() {

		motor = hardwareMap.dcMotor.get("motor_big_wheel");
		motor.setDirection(DcMotor.Direction.FORWARD);
		double power = 0;

		waitForStart();

		while (!isStopRequested()) {
			if (gamepad1.dpad_up) {
				power += 0.1;
				motor.setPower(power);
			} else if (gamepad1.dpad_down) {
				power -= 0.1;
				motor.setPower(power);
			} else if (gamepad1.dpad_left || gamepad1.dpad_right) {
				power = 0;
				motor.setPower(power);
			}

			telemetry.addData("Power", power);
			telemetry.update();

		}

	}

}
