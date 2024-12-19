package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;

/**
 * This is the PathCallback class. This class handles callbacks of Runnables in PathChains.
 * Basically, this allows you to run non-blocking code in the middle of PathChains.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class PathCallback extends SingleRunAction {

    private double startCondition;

    private int type;
    private int index;

    public static final int TIME = 0;
    public static final int PARAMETRIC = 1;

    /**
     * This creates a new PathCallback with a specified start condition (either time or parametric),
     * a Runnable of code to run (preferably a lambda statement), a type (using the class constants),
     * and an index for which Path within a PathChain the callback is to run on.
     *
     * @param startCondition This defines when the callback is to be run, either as a wait time in
     *                       milliseconds or a t-value (parametric time) point.
     * @param runnable This contains the code to run when the callback is called.
     * @param type This defines the type of callback using the class constants.
     * @param index This defines which Path within the PathChain the callback is to run on.
     */
    public PathCallback(double startCondition, Runnable runnable, int type, int index) {
        super(runnable);
        this.startCondition = startCondition;
        this.type = type;
        if (this.type != TIME || this.type != PARAMETRIC) {
            this.type = PARAMETRIC;
        }
        if (this.type == TIME && this.startCondition < 0) {
            this.startCondition = 0.0;
        }
        if (this.type == PARAMETRIC) {
            this.startCondition = MathFunctions.clamp(this.startCondition, 0, 1);
        }
        this.index = index;
    }

    /**
     * This returns the type of callback this is (time or parametric).
     *
     * @return This returns the type of callback.
     */
    public int getType() {
        return type;
    }

    /**
     * This returns the start condition for this callback. This will be the wait time in milliseconds
     * if this is a time callback or a t-value if this is a parametric callback.
     *
     * @return This returns the start condition.
     */
    public double getStartCondition() {
        return startCondition;
    }

    /**
     * This returns the index of which Path the callback is to run on within the PathChain.
     *
     * @return This returns the Path index of this callback.
     */
    public int getIndex() {
        return index;
    }
}
