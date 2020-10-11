package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

public class RobotMotionSystemTeleOpTask extends RobotMotionSystemTask {
    public double xSpeedNormalized = 0;
    public double ySpeedNormalized = 0;
    public double zRotSpeedNormalized = 0;
    protected double theoraticalMaximumLinearX, theoraticalMaximumLinearY, theoraticalMaximumAngular;

    public RobotMotionSystemTeleOpTask(){
        super();
        this.xSpeedNormalized = 0;
        this.ySpeedNormalized = 0;
        this.zRotSpeedNormalized = 0;
    }

    public RobotMotionSystemTeleOpTask(RobotMotionSystemTeleOpTask oldTask){
        super(oldTask);
        this.xSpeedNormalized = oldTask.xSpeedNormalized;
        this.ySpeedNormalized = oldTask.ySpeedNormalized;
        this.zRotSpeedNormalized = oldTask.zRotSpeedNormalized;
    }

    @Override
    protected void __startTask() {
        this.theoraticalMaximumAngular = this.getMotionSystem().calculateMaxAngularSpeedInDegPerSec();
        this.theoraticalMaximumLinearX = this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec();
        this.theoraticalMaximumLinearY = this.getMotionSystem().calculateMaxLinearYSpeedInCMPerSec();
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {
        this.getMotionSystem().setRobotSpeed(
                this.xSpeedNormalized * theoraticalMaximumLinearX,
                this.ySpeedNormalized * theoraticalMaximumLinearY,
                this.zRotSpeedNormalized * theoraticalMaximumAngular
        );
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return null;
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {

    }
}
