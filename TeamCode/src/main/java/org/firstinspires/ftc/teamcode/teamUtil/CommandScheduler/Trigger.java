package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

import java.util.function.BooleanSupplier;

public class Trigger {
	private BooleanSupplier triggerCondition;
	
	public BooleanSupplier getTriggerCondition() {
		return triggerCondition;
	}
	private boolean toggle;

	private TriggerAction
		onTrue = () -> {},
        onFalse = () -> {},
		toggleOnTrue = () -> {},
		toggleOnFalse = () -> {};


	public Trigger(BooleanSupplier triggerCondition) {
		this.triggerCondition = triggerCondition;
		Scheduler.getInstance().registerTrigger(this);
	}

	public void poll() {
		if(triggerCondition.getAsBoolean()) toggle = !toggle;
		pollOnTrue();
		pollOnFalse();
		pollToggleOnTrue();
		pollToggleOnFalse();
	}

	public Trigger or(BooleanSupplier triggerCondition){
		this.triggerCondition = () -> triggerCondition.getAsBoolean() || this.triggerCondition.getAsBoolean();
		return this;
	}

	private void pollOnTrue(){
		if(triggerCondition.getAsBoolean()) onTrue.triggerAction();
	}
	private void pollOnFalse(){
		if(!triggerCondition.getAsBoolean()) onFalse.triggerAction();
	}
	private void pollToggleOnTrue(){
		if(toggle) toggleOnTrue.triggerAction();
	}
	private void pollToggleOnFalse(){
		if(!toggle) toggleOnFalse.triggerAction();
	}
	
	public Trigger onTrue(TriggerAction onTrue) {
		this.onTrue = onTrue;
		return this;
	}
	
	public Trigger onFalse(TriggerAction onFalse) {
		this.onFalse = onFalse;
		return this;
	}
	
	public Trigger toggleOnTrue(TriggerAction toggleOnTrue) {
		this.toggleOnTrue = toggleOnTrue;
		return this;
	}
	
	public Trigger toggleOnFalse(TriggerAction toggleOnFalse) {
		this.toggleOnFalse = toggleOnFalse;
		return this;
	}
	
	public interface TriggerAction {
		void triggerAction();
	}
}