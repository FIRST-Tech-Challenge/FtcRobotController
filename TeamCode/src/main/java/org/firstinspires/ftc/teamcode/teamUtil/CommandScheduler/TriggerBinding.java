package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

public class TriggerBinding {
	Trigger.TriggerAction triggerAction;
	public TriggerBinding(Trigger.TriggerAction triggerAction){
		this.triggerAction = triggerAction;
	}

	public TriggerBinding(){
		this.triggerAction = () -> {};
	}
}
