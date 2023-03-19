package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;

@TeleOp(name="triggerTest", group="test")
public class TriggerTest extends ConfiguredOpMode {
	
	int count;
	
	@Override
	public void superInit() {
		telemetry.setAutoClear(true);
	}
	
	@Override
	public void registerTriggers() {
		gamepadEX1.a.onPress()
				.onTrue(() -> {
					count++;
				});
		
		gamepadEX1.b.isPressed()
				.onTrue(() -> telemetry.addData("boolean b", true))
				.onFalse(() -> telemetry.addData("boolean b", false));
	}
	
	@Override
	public void superInit_Loop() {
	
	}
	
	@Override
	public void superStart() {
	
	}
	
	@Override
	public void superLoop() {
		telemetry.addData("button state", gamepadEX1.x.buttonState());
		telemetry.addData("count a", count);
	}
	
	@Override
	public void superStop() {
	
	}
}
