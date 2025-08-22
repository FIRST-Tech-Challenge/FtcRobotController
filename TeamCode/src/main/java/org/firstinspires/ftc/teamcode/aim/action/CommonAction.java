package org.firstinspires.ftc.teamcode.aim.action;
import java.util.function.Supplier;

public class CommonAction extends Action {
    private Supplier<Boolean> runFunction;

    public CommonAction(String name, Supplier<Boolean> runFunction) {
        super(name);
        this.runFunction = runFunction;
    }

    @Override
    public boolean run() {
        if (runFunction == null) {
            return true;
        }
        return runFunction.get();
    }
}
