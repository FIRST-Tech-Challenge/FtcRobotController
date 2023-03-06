package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

public abstract class Subsystem {
	public Subsystem(){
		Scheduler.getInstance().registerSubsystem(this);
	}
	public abstract void init();
	public abstract void read();
	public abstract void update();
	public abstract void close();
}
