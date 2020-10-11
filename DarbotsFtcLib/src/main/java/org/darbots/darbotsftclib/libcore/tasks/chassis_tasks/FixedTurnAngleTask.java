package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

public class FixedTurnAngleTask extends RobotMotionSystemTask {
    public double angleToTurn;
    public double startAngularVelocityNormalized;
    public double endAngularVelocityNormalized;
    public double cruiseAngularVelocityNormalized;

    private double startAngle = 0;
    private double targetAngle = 0;

    public FixedTurnAngleTask(double angleToTurn, double startAngularVelocityNormalized, double cruiseAngularVelocityNormalized, double endAngularVelocityNormalized){
        this.angleToTurn = angleToTurn;
        this.startAngularVelocityNormalized = startAngularVelocityNormalized;
        this.endAngularVelocityNormalized = endAngularVelocityNormalized;
        this.cruiseAngularVelocityNormalized = cruiseAngularVelocityNormalized;
    }

    public FixedTurnAngleTask(FixedTurnAngleTask oldTask){
        this.angleToTurn = oldTask.angleToTurn;
        this.startAngularVelocityNormalized = oldTask.startAngularVelocityNormalized;
        this.cruiseAngularVelocityNormalized = oldTask.cruiseAngularVelocityNormalized;
        this.endAngularVelocityNormalized = oldTask.endAngularVelocityNormalized;
    }

    @Override
    protected void __startTask() {
        startAngle = this.getSupposedWorldTaskStartPose().getRotationZ();
        targetAngle = XYPlaneCalculations.normalizeDeg(this.startAngle + this.angleToTurn);
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {

    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return new RobotPose2D(0,0,angleToTurn);
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {

    }
}
