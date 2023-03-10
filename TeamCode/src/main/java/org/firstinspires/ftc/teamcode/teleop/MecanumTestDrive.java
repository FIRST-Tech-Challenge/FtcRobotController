package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.GamepadEX;

@TeleOp(name = "mecnum and nothing else lmao CHIMPERS", group = "uhhhhh")
public class MecanumTestDrive extends ConfiguredOpMode {
	MecanumDriveBase mecanum = new MecanumDriveBase();

	@Override
	public void superInit() {

	}

	@Override
	public void registerTriggers() {

	}

	@Override
	public void superInit_Loop() {

	}

	@Override
	public void superStart() {

	}

	@Override
	public void superLoop() {
		mecanum.trueHolonomicDrive(gamepadEX1.leftX.getValue(), gamepadEX1.leftY.getValue(), gamepadEX1.rightX.getValue());
	}

	@Override
	public void superStop() {

	}
}
