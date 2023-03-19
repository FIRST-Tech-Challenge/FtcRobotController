package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;

@TeleOp(name = "FULL WORLDS DRIVECODE, ft. CRACKER BARREL \n\n\n\n BIIIIIG", group = ".CRACKER BARREL")
public class DriveCode extends ConfiguredOpMode {
	private MecanumDriveBase mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;

	@Override
	public void superInit() {
		mecanum = new MecanumDriveBase(true);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
		limitSwitch = new LimitSwitch();
		lift = new Lift();
		telemetry.setAutoClear(true);
		mecanum.resetIMU();
	}

	@Override
	public void registerTriggers() {
		gamepadEX2.rightY.applyDeadZone(0.2);
		
		gamepadEX2.right_trigger.thresholdTrigger(0.05)
				.onTrue(() -> intake.presetTargetPosition(Intake.intakePos.OPEN))
				.onFalse(() -> intake.presetTargetPosition(Intake.intakePos.CLOSED)
				);
		
		gamepadEX1.right_bumper
				.onPress()
				.onTrue(() -> {
					mecanum.flipDriveBase();
				})
				.toggleOnTrue(() -> {
					arm.presetTargetPosition(Arm.armPos.BACK);
					wrist.presetTargetPosition(Wrist.wristPos.BACK);
				})
				.toggleOnFalse(() -> {
					arm.presetTargetPosition(Arm.armPos.FRONT);
					wrist.presetTargetPosition(Wrist.wristPos.FRONT);
				});
		
		gamepadEX1.left_bumper
				.onPress()
				.toggleOnTrue(() -> {
					mecanum.POVmode(true);
				})
				.toggleOnFalse(() -> {
					mecanum.POVmode(false);
				});
	}

	@Override
	public void superInit_Loop() {
		mecanum.resetIMU();
	}

	@Override
	public void superStart() {
	
	}

	@Override
	public void superLoop() {
		mecanum.trueHolonomicDrive(
				gamepadEX1.leftX.getValue(),
				gamepadEX1.leftY.getValue(),
				gamepadEX1.rightX.getValue(),
				gamepadEX1.rightY.getValue(),
				1 - gamepadEX1.right_trigger.getValue()
		);
		
		lift.liftInputs(
				gamepadEX2.rightY.getValue(),
				limitSwitch.limitSwitchEX.buttonState(),
				gamepadEX2.y.buttonState(),
				gamepadEX2.b.buttonState(),
				gamepadEX2.x.buttonState(),
				gamepadEX2.a.buttonState()
		);
	}

	@Override
	public void superStop() {

	}
}
