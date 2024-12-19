package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the SingleRunAction class. It handles running Runnables once, until a reset is called.
 * It also forms the basis of the PathCallback class. Basically, if you want to run a certain action
 * once despite looping through a section of code multiple times, then this is for you.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/6/2024
 */
public class SingleRunAction {
    private boolean hasBeenRun;

    private Runnable runnable;

    /**
     * This creates a new SingleRunAction with a Runnable containing the code to be run for this action.
     * The name is a slight bit misleading, as this can actually be run multiple times. However, the
     * run() method only runs once before the reset() method needs to be called to allow the
     * SingleRunAction to run again.
     *
     * @param runnable This is a Runnable containing the code to be run. Preferably, use lambda statements.
     */
    public SingleRunAction(Runnable runnable) {
        this.runnable = runnable;
    }

    /**
     * This returns if the SingleRunAction has been run yet. Running reset() will reset this.
     *
     * @return This returns if it has been run.
     */
    public boolean hasBeenRun() {
        return hasBeenRun;
    }

    /**
     * This runs the Runnable of the SingleRunAction. It will only run once before requiring a reset.
     *
     * @return This returns if the Runnable was successfully run. If not, then a reset is needed to run again.
     */
    public boolean run() {
        if (!hasBeenRun) {
            hasBeenRun = true;
            runnable.run();
            return true;
        }
        return false;
    }

    /**
     * This resets the SingleRunAction and makes it able to run again. The SingleRunAction is set
     * to "has not been run", allowing for multiple uses of the Runnable.
     */
    public void reset() {
        hasBeenRun = false;
    }
}
