package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

public class Trigger {
	private Boolean triggerCondition;

	private boolean
		toggleTrue = false,
		toggleFalse = false;

	private TriggerBinding
		onTrue = new TriggerBinding(),
        onFalse = new TriggerBinding(),
		toggleOnTrue = new TriggerBinding(),
		toggleOnFalse = new TriggerBinding();


	public Trigger(Boolean triggerCondition) {
		this.triggerCondition = triggerCondition;
		Scheduler.getInstance().registerTrigger(this);
	}

	public void poll() {
		pollOnTrue();
		pollOnFalse();
		pollToggleOnTrue();
		pollToggleOnFalse();
	}

	public Trigger or(Boolean triggerCondition){
		this.triggerCondition = triggerCondition || this.triggerCondition;
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
		if(triggerCondition) onTrue.triggerAction.triggerAction();
	}
	private void pollOnFalse(){
		if(!triggerCondition) onFalse.triggerAction.triggerAction();
	}
	private void pollToggleOnTrue(){
		if(triggerCondition) toggleTrue = !toggleTrue;
		if(toggleTrue) toggleOnTrue.triggerAction.triggerAction();
	}
	private void pollToggleOnFalse(){
		if(!triggerCondition) toggleFalse = !toggleFalse;
		if(toggleFalse) toggleOnFalse.triggerAction.triggerAction();
	}

	public interface TriggerAction {
		void triggerAction();
	}
}