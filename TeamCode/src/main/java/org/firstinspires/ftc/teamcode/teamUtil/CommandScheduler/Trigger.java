package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

import java.util.function.BooleanSupplier;

public class Trigger {
	private BooleanSupplier triggerCondition;

	private boolean
		toggleTrue = false,
		toggleFalse = false;

	private TriggerBinding
		onTrue = new TriggerBinding(),
        onFalse = new TriggerBinding(),
		toggleOnTrue = new TriggerBinding(),
		toggleOnFalse = new TriggerBinding();


	public Trigger(BooleanSupplier triggerCondition) {
		this.triggerCondition = triggerCondition;
		Scheduler.getInstance().registerTrigger(this);
	}

	public void poll() {
		pollOnTrue();
		pollOnFalse();
		pollToggleOnTrue();
		pollToggleOnFalse();
	}

	public Trigger or(BooleanSupplier triggerCondition){
		this.triggerCondition = () -> triggerCondition.getAsBoolean() || this.triggerCondition.getAsBoolean();
		return this;
	}

	public Trigger onTrue(TriggerAction triggerAction){
		this.onTrue = new TriggerBinding(triggerAction);
		return this;
	}

	public Trigger onFalse(TriggerAction triggerAction){
		this.onFalse = new TriggerBinding(triggerAction);
		return this;
	}

	public Trigger toggleOnTrue(TriggerAction triggerAction){
		this.toggleOnTrue = new TriggerBinding(triggerAction);
		return this;
	}

	public Trigger toggleOnFalse(TriggerAction triggerAction){
		this.toggleOnFalse = new TriggerBinding(triggerAction);
		return this;
	}

	private void pollOnTrue(){
		if(triggerCondition.getAsBoolean()) onTrue.triggerAction.triggerAction();
	}
	private void pollOnFalse(){
		if(!triggerCondition.getAsBoolean()) onFalse.triggerAction.triggerAction();
	}
	private void pollToggleOnTrue(){
		if(triggerCondition.getAsBoolean()) toggleTrue = !toggleTrue;
		if(toggleTrue) toggleOnTrue.triggerAction.triggerAction();
	}
	private void pollToggleOnFalse(){
		if(!triggerCondition.getAsBoolean()) toggleFalse = !toggleFalse;
		if(toggleFalse) toggleOnFalse.triggerAction.triggerAction();
	}

	public interface TriggerAction {
		void triggerAction();
	}
}