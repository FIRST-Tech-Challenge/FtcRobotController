package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Tensioner;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;

@TeleOp(name="Tensioner Control", group = "Tensioner")
public class TensionerControl extends ConfiguredOpMode {
	private Tensioner tensioner;
	
	@Override
	public void superInit() {
		tensioner = new Tensioner(Tensioner.RunMode.FREE);
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
		tensioner.tensionerInputs(gamepadEX1.rightY.getValue());
	}
	
	@Override
	public void superStop() {
	
	}
}
