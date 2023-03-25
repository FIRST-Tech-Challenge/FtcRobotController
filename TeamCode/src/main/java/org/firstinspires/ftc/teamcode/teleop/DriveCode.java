package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Trigger;

@TeleOp(name = "FULL WORLDS DRIVECODE, ft. CRACKER BARREL \n\n\n\n BIIIIIG", group = ".CRACKER BARREL")
public class DriveCode extends ConfiguredOpMode {
	private MecanumDriveBase mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	
	Trigger bothBumpers;
	
	
	@Override
	public void superInit() {
		mecanum = new MecanumDriveBase(true);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
		limitSwitch = new LimitSwitch();
		lift = new Lift();
	}

	@Override
	public void registerTriggers() {
		gamepadEX2.rightY.applyDeadZone(0.2);
		
		gamepadEX2.right_trigger.thresholdTrigger(0.05)
				.onTrue(() -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.onFalse(() -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				});
		
		gamepadEX1.left_bumper
				.onPress()
				.onTrue(() -> {
					mecanum.setDriveBaseFlip(true);
				});
		gamepadEX1.right_bumper
				.onPress()
				.onTrue(() -> {
					mecanum.setDriveBaseFlip(false);
				});
		
		gamepadEX1.left_bumper
				.isPressed()
				.onTrue(() -> {
					r.setPickup(false);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				});
		gamepadEX1.right_bumper
				.isPressed()
				.onTrue(() -> {
					r.setPickup(false);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				});
		
		bothBumpers = new Trigger(() -> (gamepadEX1.right_bumper.buttonState() || gamepadEX1.left_bumper.buttonState()))
				.onFalse(() -> {
					r.setPickup(true);
				});
		
		gamepadEX1.right_stick_button
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
		
		telemetry.addData("delivery", r.isDelivery());
		telemetry.addData("pickup", r.isPickup());
	}

	@Override
	public void superStop() {

	}
}
