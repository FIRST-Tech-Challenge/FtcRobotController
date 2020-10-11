package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public class RobotMotionSystemTeleOpPIDTask extends RobotMotionSystemTeleOpTask {
    protected RobotPose2D lastRelativePose;
    protected RobotVector2D lastSpeed;
    protected ElapsedTime timeBetweenUpdateStatusCalls;

    public RobotMotionSystemTeleOpPIDTask(){
        super();
    }

    public RobotMotionSystemTeleOpPIDTask(RobotMotionSystemTeleOpTask oldTask){
        super(oldTask);
    }

    @Override
    protected void __startTask(){
        super.__startTask();
        this.timeBetweenUpdateStatusCalls = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.lastRelativePose = new RobotPose2D(0,0,0);
        this.lastSpeed = new RobotVector2D(0,0,0);
    }

    @Override
    protected void __updateStatus() {
        double timeInBetween = timeBetweenUpdateStatusCalls.seconds();
        timeBetweenUpdateStatusCalls.reset();

        RobotPose2D currentRelative = __calcCurrentRelative(timeInBetween);

        RobotVector2D wantedSpeed = new RobotVector2D(
                this.xSpeedNormalized * theoraticalMaximumLinearX,
                this.ySpeedNormalized * theoraticalMaximumLinearY,
                this.zRotSpeedNormalized * theoraticalMaximumAngular
        );
        RobotVector2D actualSpeed = this.setRobotSpeed(wantedSpeed,currentRelative);

        this.lastSpeed.setValues(actualSpeed);
        this.lastRelativePose.setValues(currentRelative);
    }

    protected RobotPose2D __calcCurrentRelative(double deltaTime){
        RobotPose2D lastRelativeMoved = new RobotPose2D(
                lastSpeed.X * deltaTime,
                lastSpeed.Y * deltaTime,
                lastSpeed.getRotationZ() * deltaTime
        );
        RobotPose2D currentRelative = XYPlaneCalculations.getAbsolutePosition(lastRelativePose,lastRelativeMoved);
        return currentRelative;
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return this.__calcCurrentRelative(timeBetweenUpdateStatusCalls.seconds());
    }
}
