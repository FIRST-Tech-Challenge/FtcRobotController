/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control.ChassisPIDCalculator;
import org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control.PIDCoefficients;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;

import java.util.ArrayList;

//distanceFactor = wantedDistance / actualDistance
public abstract class RobotMotionSystem implements RobotNonBlockingDevice {
    public final static PIDCoefficients LINEAR_X_PID_DEFAULT = new PIDCoefficients(5,0,1);
    public final static PIDCoefficients LINEAR_Y_PID_DEFAULT = new PIDCoefficients(5,0,1);
    public final static PIDCoefficients ROTATIONAL_Z_PID_DEFAULT = new PIDCoefficients(3,0,0.2);

    private ArrayList<RobotMotionSystemTask> m_TaskLists;
    private Robot2DPositionTracker m_PosTracker;
    private double m_LinearYMotionDistanceFactor;
    private double m_LinearXMotionDistanceFactor;
    private double m_RotationalMotionDistanceFactor;
    private boolean m_PosTrackerIsAsync;
    private PIDCoefficients m_LinearXPIDCoefficient, m_LinearYPIDCoefficient, m_RotationalPIDCoefficient;
    private RobotPose2D m_LastTaskFinishFieldPos;
    private ChassisPIDCalculator m_PIDCalculator;
    private double m_Cache_MaxLinearX, m_Cache_MaxLinearY, m_Cache_MaxZRot, m_Cache_MaxLinear;
    private RobotPose2D m_Cache_WorldPosition = null;

    private double m_DrawRobotHalfWidth = 9 / XYPlaneCalculations.INCH_PER_CM, m_DrawRobotHalfLength = 9 / XYPlaneCalculations.INCH_PER_CM;
    private RobotPoint2D m_RobotFrontPoint = new RobotPoint2D(m_DrawRobotHalfLength,0);
    public String robotDrawColor = "#000066",pathDrawColor = "#0066ff", supposedPoseDrawColor = "#cccccc";

    public RobotMotionSystem(Robot2DPositionTracker PositionTracker){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = PositionTracker;
        this.setLinearMotionDistanceFactor(1);
        this.m_RotationalMotionDistanceFactor = 1;
        this.m_LinearXPIDCoefficient = LINEAR_X_PID_DEFAULT;
        this.m_LinearYPIDCoefficient = LINEAR_Y_PID_DEFAULT;
        this.m_RotationalPIDCoefficient = ROTATIONAL_Z_PID_DEFAULT;
        this.m_PIDCalculator = new ChassisPIDCalculator(this.m_LinearXPIDCoefficient,this.m_LinearYPIDCoefficient,this.m_RotationalPIDCoefficient);
        this.m_LastTaskFinishFieldPos = null;
        this.m_Cache_MaxLinear = -1;
        this.m_Cache_MaxLinearX = -1;
        this.m_Cache_MaxLinearY = -1;
        this.m_Cache_MaxZRot = -1;

        if(PositionTracker != null && PositionTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
    }
    public RobotMotionSystem(RobotMotionSystem MotionSystem){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = MotionSystem.m_PosTracker;
        this.m_LinearYMotionDistanceFactor = MotionSystem.m_LinearYMotionDistanceFactor;
        this.m_LinearXMotionDistanceFactor = MotionSystem.m_LinearXMotionDistanceFactor;
        this.m_RotationalMotionDistanceFactor = MotionSystem.m_RotationalMotionDistanceFactor;
        this.m_LinearXPIDCoefficient = MotionSystem.m_LinearXPIDCoefficient;
        this.m_LinearYPIDCoefficient = MotionSystem.m_LinearYPIDCoefficient;
        this.m_RotationalPIDCoefficient = MotionSystem.m_RotationalPIDCoefficient;
        this.m_PIDCalculator = new ChassisPIDCalculator(this.m_LinearXPIDCoefficient,this.m_LinearYPIDCoefficient,this.m_RotationalPIDCoefficient);
        this.m_LastTaskFinishFieldPos = MotionSystem.m_LastTaskFinishFieldPos == null ? null : new RobotPose2D(MotionSystem.m_LastTaskFinishFieldPos);
        this.m_Cache_MaxLinear = MotionSystem.m_Cache_MaxLinear;
        this.m_Cache_MaxLinearX = MotionSystem.m_Cache_MaxLinearX;
        this.m_Cache_MaxLinearY = MotionSystem.m_Cache_MaxLinearY;
        this.m_Cache_MaxZRot = MotionSystem.m_Cache_MaxZRot;
        this.m_Cache_WorldPosition = MotionSystem.m_Cache_WorldPosition;

        if(MotionSystem.m_PosTracker != null && MotionSystem.m_PosTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
    }

    public double getDrawRobotWidth(){
        return this.m_DrawRobotHalfWidth * 2.0;
    }

    public void setDrawRobotWidth(double widthInCM){
        this.m_DrawRobotHalfWidth = widthInCM / 2.0;
    }

    public double getDrawRobotLength(){
        return this.m_DrawRobotHalfLength * 2.0;
    }

    public void setDrawRobotLength(double lengthInCM){
        this.m_DrawRobotHalfLength = lengthInCM / 2.0;
        m_RobotFrontPoint.X = lengthInCM / 2.0;
    }

    public ChassisPIDCalculator getPIDCalculator(){
        return this.m_PIDCalculator;
    }

    public double getLinearYMotionDistanceFactor(){
        return this.m_LinearYMotionDistanceFactor;
    }
    public void setLinearYMotionDistanceFactor(double Factor){
        this.m_LinearYMotionDistanceFactor = Factor;
        this.m_Cache_MaxLinearY = -1;
        this.m_Cache_MaxLinear = -1;
    }

    public double getLinearXMotionDistanceFactor(){
        return this.m_LinearXMotionDistanceFactor;
    }
    public void setLinearXMotionDistanceFactor(double Factor){
        this.m_LinearXMotionDistanceFactor = Factor;
        this.m_Cache_MaxLinearX = -1;
        this.m_Cache_MaxLinear = -1;
    }
    public void setLinearMotionDistanceFactor(double Factor){
        this.setLinearYMotionDistanceFactor(Factor);
        this.setLinearXMotionDistanceFactor(Factor);
    }

    public double getRotationalMotionDistanceFactor(){
        return this.m_RotationalMotionDistanceFactor;
    }

    public void setRotationalMotionDistanceFactor(double Factor){
        this.m_RotationalMotionDistanceFactor = Factor;
        this.m_Cache_MaxZRot = -1;
    }

    public PIDCoefficients getLinearXPIDCoefficient(){
        return this.m_LinearXPIDCoefficient;
    }

    public void setLinearXPIDCoefficient(@NonNull PIDCoefficients LinearXPID){
        this.m_LinearXPIDCoefficient = LinearXPID;
        this.m_PIDCalculator.xPIDCoefficients = LinearXPID;
    }

    public PIDCoefficients getLinearYPIDCoefficient(){
        return this.m_LinearYPIDCoefficient;
    }

    public void setLinearYPIDCoefficient(@NonNull PIDCoefficients LinearYPID){
        this.m_LinearYPIDCoefficient = LinearYPID;
        this.m_PIDCalculator.yPIDCoefficients = LinearYPID;
    }

    public void setLinearPIDCoefficients(@NonNull PIDCoefficients LinearPIDs){
        this.m_LinearXPIDCoefficient = LinearPIDs;
        this.m_LinearYPIDCoefficient = LinearPIDs;
        this.m_PIDCalculator.xPIDCoefficients = LinearPIDs;
        this.m_PIDCalculator.yPIDCoefficients = LinearPIDs;
    }

    public PIDCoefficients getRotationalPIDCoefficient(){
        return this.m_RotationalPIDCoefficient;
    }

    public void setRotationalPIDCoefficient(@NonNull PIDCoefficients RotationalPID){
        this.m_RotationalPIDCoefficient = RotationalPID;
        this.m_PIDCalculator.rotZPIDCoefficients = RotationalPID;
    }

    public double getPreferredAngle(double preferredWorldAngle){
        double currentWorldAngle = 0;
        if(this.getLastTaskFinishFieldPos() != null && (!Double.isNaN(this.getLastTaskFinishFieldPos().getRotationZ()))){
            currentWorldAngle = this.getLastTaskFinishFieldPos().getRotationZ();
        }else{
            currentWorldAngle = this.getCurrentPosition().getRotationZ();
        }
        double preferredAngle = XYPlaneCalculations.normalizeDeg(preferredWorldAngle - currentWorldAngle);
        return preferredAngle;
    }

    public Robot2DPositionTracker getPositionTracker(){
        return this.m_PosTracker;
    }
    public void setPositionTracker(Robot2DPositionTracker PositionTracker){
        this.m_PosTracker = PositionTracker;

        if(PositionTracker != null && PositionTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
        this.clearPositionCache();
    }
    public void addTask(@NonNull RobotMotionSystemTask Task){
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void replaceTask(@NonNull RobotMotionSystemTask Task){
        if(!this.m_TaskLists.isEmpty()){
             if(this.m_TaskLists.get(0).isBusy())
                 this.m_TaskLists.get(0).userIntentionalDelete = true;
                 this.m_TaskLists.get(0).stopTask();
        }
        this.m_TaskLists.clear();
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void deleteCurrentTask(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        this.m_TaskLists.get(0).userIntentionalDelete = true;
        this.m_TaskLists.get(0).stopTask();
        this.m_TaskLists.remove(0);
        this.scheduleTasks();
    }

    public ArrayList<RobotMotionSystemTask> getTaskLists(){
        return this.m_TaskLists;
    }

    public RobotMotionSystemTask getCurrentTask(){
        return this.m_TaskLists.isEmpty() ? null : this.m_TaskLists.get(0);
    }

    public void deleteAllTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        RobotMotionSystemTask currentTask = this.m_TaskLists.get(0);
        currentTask.userIntentionalDelete = true;
        currentTask.stopTask();
        this.m_TaskLists.clear();
        this.__stopMotion();
    }

    public void __checkTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }else if(!this.m_TaskLists.get(0).isBusy()){
            this.deleteCurrentTask();
        }
    }

    protected abstract void __stopMotion();

    protected void scheduleTasks(){
        if(!this.m_TaskLists.isEmpty()){
            if(!this.m_TaskLists.get(0).isBusy()) {
                this.m_TaskLists.get(0).setMotionSystem(this);
                this.m_TaskLists.get(0).startTask();
            }
        }else{
            this.__stopMotion();
        }
    }

    @Override
    public boolean isBusy(){
        return (!this.m_TaskLists.isEmpty());
    }

    @Override
    public void updateStatus(){
        if(this.m_PosTrackerIsAsync){
            RobotNonBlockingDevice NonBlockingTracker = (RobotNonBlockingDevice) this.m_PosTracker;
            NonBlockingTracker.updateStatus();
        }
        this.__updateMotorStatus();
        this.clearPositionCache();
        if((!this.m_TaskLists.isEmpty())){
            this.m_TaskLists.get(0).updateStatus();
        }
    }

    public void clearPositionCache(){
        this.m_Cache_WorldPosition = this.getPositionTracker().getCurrentPosition();
    }

    protected abstract void __updateMotorStatus();

    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }

    public RobotPose2D getLastTaskFinishFieldPos(){
        return this.m_LastTaskFinishFieldPos;
    }

    public void setLastTaskFinishFieldPos(RobotPose2D fieldPos){
        if(fieldPos == null){
            this.m_LastTaskFinishFieldPos = null;
        }
        if(this.m_LastTaskFinishFieldPos != null){
            this.m_LastTaskFinishFieldPos.setValues(fieldPos);
        }else{
            this.m_LastTaskFinishFieldPos = new RobotPose2D(fieldPos);
        }
    }

    public void stop(){
        this.deleteAllTasks();
    }

    public void terminate(){
        this.stop();
        if(this.getPositionTracker() != null){
            this.getPositionTracker().stop();
        }
    }

    public abstract RobotVector2D getTheoreticalMaximumMotionState(double WantedXSpeedInCMPerSec, double WantedYSpeedInCMPerSec, double WantedZRotSpeedInDegPerSec);
    public RobotVector2D getTheoreticalMaximumMotionState(RobotVector2D WantedVelocityVector){
        return this.getTheoreticalMaximumMotionState(WantedVelocityVector.X,WantedVelocityVector.Y,WantedVelocityVector.getRotationZ());
    }
    protected abstract void __setRobotSpeed(double XSpeedInCMPerSec, double YSpeedInCMPerSec, double ZRotSpeedInDegPerSec);
    public RobotVector2D setRobotSpeed(RobotVector2D VelocityVector){
        return this.setRobotSpeed(VelocityVector.X,VelocityVector.Y,VelocityVector.getRotationZ());
    }
    public RobotVector2D setRobotSpeed(double XSpeedInCMPerSec, double YSpeedInCMPerSec, double ZRotSpeedInDegPerSec){
        RobotVector2D biggestSpeed = this.getTheoreticalMaximumMotionState(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
        if(Math.abs(XSpeedInCMPerSec) > Math.abs(biggestSpeed.X) || Math.abs(YSpeedInCMPerSec) > Math.abs(biggestSpeed.Y) || Math.abs(ZRotSpeedInDegPerSec) > Math.abs(biggestSpeed.getRotationZ())){
            this.__setRobotSpeed(biggestSpeed.X,biggestSpeed.Y,biggestSpeed.getRotationZ());
            return biggestSpeed;
        }
        this.__setRobotSpeed(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
        return new RobotVector2D(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
    }
    public abstract void __setNormalizedRobotSpeed(double XSpeed, double YSpeed, double ZRotSpeed);

    public RobotVector2D setNormalizedRobotSpeed(double XSpeed, double YSpeed, double ZRotSpeed){
        double newXSpeed = XSpeed * this.getLinearXMotionDistanceFactor();
        double newYSpeed = YSpeed * this.getLinearYMotionDistanceFactor();
        double newRotSpeed = ZRotSpeed * this.getRotationalMotionDistanceFactor();
        this.__setNormalizedRobotSpeed(newXSpeed,newYSpeed, newRotSpeed);
        return new RobotVector2D(newXSpeed,newYSpeed,newRotSpeed);
    }
    public double[] calculateWheelAngularSpeeds(double RobotXSpeedInCMPerSec, double RobotYSpeedInCMPerSec, double RobotZRotSpeedInDegPerSec){
        return calculateRawWheelAngularSpeeds(RobotXSpeedInCMPerSec * this.getLinearXMotionDistanceFactor(), RobotYSpeedInCMPerSec * this.getLinearYMotionDistanceFactor(), RobotZRotSpeedInDegPerSec * this.getRotationalMotionDistanceFactor());
    }
    public double[] calculateWheelAngularSpeeds(RobotPose2D RobotVelocity){
        return this.calculateWheelAngularSpeeds(RobotVelocity.X,RobotVelocity.Y,RobotVelocity.getRotationZ());
    }
    public abstract double[] calculateRawWheelAngularSpeeds(double RobotXSpeedInCMPerSec, double RobotYSpeedInCMPerSec, double RobotZRotSpeedInDegPerSec);

    public RobotVector2D calculateRobotSpeed(double[] wheelSpeeds){
        RobotVector2D rawRobotSpeed = this.calculateRawRobotSpeed(wheelSpeeds);
        rawRobotSpeed.X /= this.getLinearXMotionDistanceFactor();
        rawRobotSpeed.Y /= this.getLinearYMotionDistanceFactor();
        rawRobotSpeed.setRotationZ(rawRobotSpeed.getRotationZ() / this.getRotationalMotionDistanceFactor());
        return rawRobotSpeed;
    }
    public abstract RobotVector2D calculateRawRobotSpeed(double[] wheelSpeeds);
    public double calculateMaxLinearXSpeedInCMPerSec(){
        if(this.m_Cache_MaxLinearX == -1) {
            this.m_Cache_MaxLinearX = this.calculateMaxLinearXSpeedInCMPerSec(0);
        }
        return this.m_Cache_MaxLinearX;
    }
    public double calculateMaxLinearYSpeedInCMPerSec(){
        if(this.m_Cache_MaxLinearY == -1){
            this.m_Cache_MaxLinearY = this.calculateMaxLinearYSpeedInCMPerSec(0);
        }
        return this.m_Cache_MaxLinearY;
    }
    public double calculateMaxAngularSpeedInDegPerSec(){
        if(this.m_Cache_MaxZRot == -1) {
            this.m_Cache_MaxZRot = this.calculateMaxAngularSpeedInDegPerSec(0, 0);
        }
        return this.m_Cache_MaxZRot;
    }
    public double calculateMaxLinearXSpeedInCMPerSec(double angularSpeedInDegPerSec){
        return this.calculateRawMaxLinearXSpeedInCMPerSec(angularSpeedInDegPerSec * this.getRotationalMotionDistanceFactor()) / this.getLinearXMotionDistanceFactor();
    }
    public abstract double calculateRawMaxLinearXSpeedInCMPerSec(double angularSpeedInDegPerSec);

    public double calculateMaxLinearYSpeedInCMPerSec(double angularSpeedInDegPerSec){
        return this.calculateRawMaxLinearYSpeedInCMPerSec(angularSpeedInDegPerSec * this.getRotationalMotionDistanceFactor()) / this.getLinearYMotionDistanceFactor();
    }

    public abstract double calculateRawMaxLinearYSpeedInCMPerSec(double angularSpeedInDegPerSec);

    public double calculateMaxAngularSpeedInDegPerSec(double xSpeedInCMPerSec, double ySpeedInCMPerSec){
        return this.calculateRawMaxAngularSpeedInDegPerSec(xSpeedInCMPerSec * this.getLinearXMotionDistanceFactor(),ySpeedInCMPerSec * this.getLinearYMotionDistanceFactor()) / this.getRotationalMotionDistanceFactor();
    }

    public abstract double calculateRawMaxAngularSpeedInDegPerSec(double xSpeedInCMPerSec, double ySpeedInCMPerSec);

    public double calculateMaxLinearSpeedInCMPerSec() {
        if (this.m_Cache_MaxLinear == -1) {
            this.m_Cache_MaxLinear = calculateMaxLinearSpeedInCMPerSec(45);
        }
        return this.m_Cache_MaxLinear;
    }
    public double calculateMaxLinearSpeedInCMPerSec(double headingAng){
        headingAng = XYPlaneCalculations.normalizeDeg(headingAng);
        double deltaX, deltaY;
        if(headingAng == 90 || headingAng == -90) {
            return this.calculateMaxLinearYSpeedInCMPerSec();
        }else if(headingAng == 0 || headingAng == -180){ //No 180 deg here because it is normalized to [-180, 180)
            return this.calculateMaxLinearXSpeedInCMPerSec();
        }else{
            deltaX = 1;
            deltaY = Math.tan(Math.toRadians(headingAng)) * deltaX;
        }
        RobotVector2D theoraticalMax = getTheoreticalMaximumMotionState(deltaX,deltaY,0);
        return Math.hypot(theoraticalMax.X,theoraticalMax.Y);
    }
    public MotionSystemConstraints getMotionSystemConstraints(double maximumAcceleration, double maximumJerk, double maximumAngularAcceleration, double maximumAngularJerk){
        return new MotionSystemConstraints(this.calculateMaxLinearSpeedInCMPerSec(),maximumAcceleration,maximumJerk,this.calculateMaxAngularSpeedInDegPerSec(),maximumAngularAcceleration,maximumAngularJerk);
    }
    public RobotPose2D getCurrentPosition(){
        if(this.m_Cache_WorldPosition == null){
            this.clearPositionCache();
        }
        return this.m_Cache_WorldPosition;
    }

    public void drawFieldOverlay(Canvas canvas){
        canvas.setFill(this.pathDrawColor);
        canvas.setStroke(this.pathDrawColor);
        if(this.isBusy()){
            this.m_TaskLists.get(0).drawPath(canvas);
        }
        RobotPose2D currentRobotPosition = this.getCurrentPosition();
        RobotPoint2D[] robotAxisPoints = XYPlaneCalculations.getRobotExtremeBoundingBox(this.m_DrawRobotHalfLength,this.m_DrawRobotHalfLength,this.m_DrawRobotHalfWidth,this.m_DrawRobotHalfWidth);
        RobotPoint2D tempRBPoint = robotAxisPoints[3];
        robotAxisPoints[3] = robotAxisPoints[2];
        robotAxisPoints[2] = tempRBPoint;
        double[] currentRobotX = new double[4], currentRobotY = new double[4];
        for(int i = 0; i < 4; i++){
            RobotPoint2D transferredPoint = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,robotAxisPoints[i]);
            transferredPoint.X *= XYPlaneCalculations.INCH_PER_CM;
            transferredPoint.Y *= XYPlaneCalculations.INCH_PER_CM;
            currentRobotX[i] = transferredPoint.X;
            currentRobotY[i] = transferredPoint.Y;
        }
        RobotPoint2D robotFrontPoint = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,this.m_RobotFrontPoint);
        RobotPoint2D currentRobotPositionInch = new RobotPoint2D(currentRobotPosition.X * XYPlaneCalculations.INCH_PER_CM, currentRobotPosition.Y * XYPlaneCalculations.INCH_PER_CM);
        robotFrontPoint.X *= XYPlaneCalculations.INCH_PER_CM;
        robotFrontPoint.Y *= XYPlaneCalculations.INCH_PER_CM;
        canvas.setStroke(this.robotDrawColor);
        canvas.setFill(this.robotDrawColor);
        canvas.strokePolygon(currentRobotX,currentRobotY);
        canvas.strokeLine(currentRobotPositionInch.X,currentRobotPositionInch.Y,robotFrontPoint.X,robotFrontPoint.Y);
    }
}
