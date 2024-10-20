package org.firstinspires.ftc.teamcode.NewStuff.actions;

import org.firstinspires.ftc.teamcode.NewStuff.modules.Outtake;

public class DropPixelAction extends Action {

    Outtake outtake;
    ActionSet outer;

    public DropPixelAction(Action dependentAction, Outtake outtake) {
        this.dependentAction = dependentAction;
        this.outtake = outtake;
        outer = new ActionSet();
        outer.scheduleSequential(new MoveLSAction(500, outtake));
        outer.scheduleSequential(new PivotTrayAction(PivotTrayAction.TRAY_OUTTAKE_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.4));
        outer.scheduleSequential(new MoveTrayClampAction(MoveTrayClampAction.CLAMP_OPEN_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.6));
        outer.scheduleSequential(new PivotTrayAction(PivotTrayAction.TRAY_INTAKE_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.2));
        outer.scheduleSequential(new MoveLSAction(-500, outtake));
    }

    public DropPixelAction(Outtake outtake) {
        this.dependentAction = new DoneStateAction();
        this.outtake = outtake;
        outer = new ActionSet();
        outer.scheduleSequential(new MoveLSAction(500, outtake));
        outer.scheduleSequential(new PivotTrayAction(PivotTrayAction.TRAY_OUTTAKE_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.4));
        outer.scheduleSequential(new MoveTrayClampAction(MoveTrayClampAction.CLAMP_OPEN_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.6));
        outer.scheduleSequential(new PivotTrayAction(PivotTrayAction.TRAY_INTAKE_POS, outtake));
        outer.scheduleSequential(new WaitAction(0.2));
        outer.scheduleSequential(new MoveLSAction(-500, outtake));
    }

    @Override
    boolean checkDoneCondition() {
        return outer.getIsDone();
    }

    @Override
    void update() {
        outer.updateCheckDone();
    }
}
