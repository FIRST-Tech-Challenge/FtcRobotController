package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Tensioner;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;

@Autonomous(name="Tensioner Stow", group = "Tensioner")
public class TensionerStow extends ConfiguredOpMode {
	private Tensioner tensioner;
	
	@Override
	public void superInit() {
		tensioner = new Tensioner(Tensioner.RunMode.STOW);
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
	
	}
	
	@Override
	public void superStop() {
	
	}
}
