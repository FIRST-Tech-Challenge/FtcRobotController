package org.firstinspires.ftc.teamcode.dcs15815.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

@Utility(name = "Servo Test", description = "")
public class ServoTestOpMode extends LinearOpMode {
	Servo servo;

	@Override
	public void runOpMode() {

		servo = hardwareMap.servo.get("gate_servo");
		double position = 0;

		waitForStart();
		servo.setPosition(position);
		while (!isStopRequested()) {
			if (gamepad1.dpad_left) {
				position = 0;
				servo.setPosition(position);
			} else if (gamepad1.dpad_up) {
				position += .1;
				if (position > 1) {
					position = 1;
				}
				servo.setPosition(position);
			} else if (gamepad1.dpad_right) {
				position = 1;
				servo.setPosition(position);
			} else if (gamepad1.dpad_down) {
				position -= .1;
				if (position < 0) {
					position = 0;
				}
				servo.setPosition(position);

			}


			telemetry.addData("Position", position);
			telemetry.update();

		}

	}

}
