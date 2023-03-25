package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class Tensioner extends Subsystem {
	private final RobotConfig r;
	private CRServo tensioner;
	
	private double power;
	
	public Tensioner(RobotConfig r, RunMode runMode){
		this.r = r;
		this.runMode = runMode;
	}
	
	public Tensioner(RunMode runMode){
		this(RobotConfig.getInstance(), runMode);
	}
	
	public enum RunMode{
		DEPLOY,
		STOW,
		FREE
	}
	
	private RunMode runMode;
	
	@Override
	public void init() {
		tensioner = r.opMode.hardwareMap.get(CRServo.class, ConfigNames.tensioner);
	}
	
	@Override
	public void read() {
		switch (runMode){
			
			case DEPLOY:
				if(RobotConfig.elapsedTime.time() < 8.5){
					power = 1.0;
				}
				else {
					power = 0;
				}
				break;
			case STOW:
				if(RobotConfig.elapsedTime.time() < 8.0){
					power = -1.0;
				}
				else {
					power = 0;
				}
				break;
			case FREE:
				break;
		}
		
	}
	
	@Override
	public void update() {
		tensioner.setPower(power);
	}
	
	@Override
	public void close() {
	
	}
	
	public void tensionerInputs(double power){
		if (runMode == RunMode.FREE){
			this.power = power;
		}
	}
}
