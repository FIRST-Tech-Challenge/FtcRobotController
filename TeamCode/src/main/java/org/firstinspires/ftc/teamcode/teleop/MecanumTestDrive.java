package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.GamepadEX;

@TeleOp(name = "mecnum and nothing else lmao CHIMPERS", group = "uhhhhh")
public class MecanumTestDrive extends ConfiguredOpMode {
	MecanumDriveBase mecanum;
	GamepadEX gamepadEX1;

	@Override
	public void superInit() {
		r.initSystems(
				RobotConstants.configuredSystems.MECANUM
		);

		mecanum = r.getSubsystem(RobotConstants.configuredSystems.MECANUM);
		gamepadEX1 = r.getSubsystem(RobotConstants.configuredSystems.GAMEPADEX_1);
	}

	@Override
	public void superInit_Loop() {

	}

	@Override
	public void superStart() {

	}

	@Override
	public void superLoop() {
		mecanum.trueHolonomicDrive(gamepadEX1.leftX.value(), gamepadEX1.leftY.value(), gamepadEX1.rightX.value());
	}

	@Override
	public void superStop() {

	}
}
