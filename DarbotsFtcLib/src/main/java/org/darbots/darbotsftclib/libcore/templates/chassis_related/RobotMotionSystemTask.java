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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    public final static float CONST_TASKSTARTANG_NOTSET = -10000.0f;

    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private ElapsedTime m_TaskTimer;
    private RobotPose2D m_TaskActualStartFieldPos = null;
    private RobotPose2D m_TaskSupposedStartFieldPos = null;
    public RobotMotionSystemTaskCallBack TaskCallBack = null;
    protected RobotPose2D m_LastSupposedPose = null;
    protected RobotPose2D m_LastError = null;
    public boolean userIntentionalDelete = false;

    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
        this.m_TaskTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.userIntentionalDelete = false;
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task) {
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
        this.m_TaskTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.userIntentionalDelete = false;
    }
    public RobotMotionSystem getMotionSystem(){
        return this.m_MotionSystem;
    }
    public void setMotionSystem(@NonNull RobotMotionSystem MotionSystem){
        this.m_MotionSystem = MotionSystem;
    }

    public void startTask(){
        if(this.m_IsWorking){
            return;
        }
        this.m_IsWorking = true;

        this.m_LastSupposedPose = new RobotPose2D(0,0,0);
        this.m_TaskActualStartFieldPos = this.m_MotionSystem.getCurrentPosition();
        this.m_TaskSupposedStartFieldPos = this.m_MotionSystem.getLastTaskFinishFieldPos();
        if(this.m_TaskSupposedStartFieldPos == null){
            this.m_TaskSupposedStartFieldPos = new RobotPose2D(this.m_TaskActualStartFieldPos);
        }

        this.m_TaskTimer.reset();
        this.__startTask();
    }

    protected abstract void __startTask();
    protected abstract void __taskFinished();
    protected abstract void __updateStatus();
    protected abstract RobotPose2D __getSupposedTaskFinishPos();

    public void stopTask(){
        if(!this.m_IsWorking){
            return;
        }
        this.m_IsWorking = false;
        this.__taskFinished();

        RobotPose2D supposedFinishRelativeOffset;
        if(userIntentionalDelete){
            supposedFinishRelativeOffset = this.__getSupposedTaskFinishPos();
        }else{
            supposedFinishRelativeOffset = null;
        }
        RobotPose2D RelativePosMoved = this.getRelativePositionOffsetSinceStart();
        RobotPose2D CurrentFieldPos = this.m_MotionSystem.getCurrentPosition();

        if(supposedFinishRelativeOffset != null) {
            if (Double.isNaN(supposedFinishRelativeOffset.X) || Double.isInfinite(supposedFinishRelativeOffset.X)) {
                supposedFinishRelativeOffset.X = RelativePosMoved.X;
            }
            if (Double.isNaN(supposedFinishRelativeOffset.Y) || Double.isInfinite(supposedFinishRelativeOffset.Y)) {
                supposedFinishRelativeOffset.Y = RelativePosMoved.Y;
            }
            if (Double.isNaN(supposedFinishRelativeOffset.getRotationZ()) || Double.isInfinite(supposedFinishRelativeOffset.getRotationZ())) {
                supposedFinishRelativeOffset.setRotationZ(RelativePosMoved.getRotationZ());
            }
        }

        if(this.TaskCallBack != null){
            this.TaskCallBack.taskFinished(this.m_MotionSystem,this.m_TaskSupposedStartFieldPos,CurrentFieldPos,RelativePosMoved,supposedFinishRelativeOffset);
        }

        RobotPose2D supposedEndFieldPos = null;
        if(supposedFinishRelativeOffset != null){
            supposedEndFieldPos = XYPlaneCalculations.getAbsolutePosition(this.m_TaskSupposedStartFieldPos,supposedFinishRelativeOffset);
        }else{
            supposedEndFieldPos = CurrentFieldPos;
        }

        this.m_MotionSystem.setLastTaskFinishFieldPos(supposedEndFieldPos);
        this.m_MotionSystem.__checkTasks();
    }
    @Override
    public boolean isBusy(){
        return this.m_IsWorking;
    }
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

    public void updateStatus(){
        if(this.isBusy()){
            this.__updateStatus();
        }
    }

    public double getSecondsSinceTaskStart(){
        return this.m_TaskTimer.seconds();
    }
    public int getMSSinceTaskStart(){
        return (int) this.m_TaskTimer.milliseconds();
    }
    protected RobotPose2D getRelativePositionOffsetRawSinceStart(){
        return XYPlaneCalculations.getRelativePosition(
                this.m_TaskActualStartFieldPos,
                this.getMotionSystem().getCurrentPosition()
        );
    }
    protected RobotPose2D getRelativePositionOffsetSinceStart(){
        return XYPlaneCalculations.getRelativePosition(
                this.m_TaskSupposedStartFieldPos,
                this.getMotionSystem().getCurrentPosition()
        );
    }

    protected RobotPose2D getActualWorldTaskStartPose(){
        return this.m_TaskActualStartFieldPos;
    }

    protected RobotPose2D getSupposedWorldTaskStartPose(){
        return this.m_TaskSupposedStartFieldPos;
    }

    public RobotVector2D getErrorCorrectionVelocityVector(RobotPose2D currentOffset, double errorX, double errorY, double errorRotZ){
        RobotPose2D error = new RobotPose2D(errorX,errorY,errorRotZ);
        RobotPose2D supposedPosition = XYPlaneCalculations.getAbsolutePosition(currentOffset,error);
        this.m_LastSupposedPose = supposedPosition;
        this.m_LastError = error;

        this.m_MotionSystem.getPIDCalculator().feedError(errorX,errorY,errorRotZ);
        RobotVector2D correctionVelocity = this.m_MotionSystem.getPIDCalculator().getPIDPower();
        return correctionVelocity;
    }

    protected RobotVector2D getErrorCorrectionVelocityVector(RobotPose2D supposedPosition){
        return this.getErrorCorrectionVelocityVector(supposedPosition,this.getRelativePositionOffsetSinceStart());
    }

    protected RobotVector2D getErrorCorrectionVelocityVector(RobotPose2D supposedPosition, RobotPose2D relativePosition){
        this.m_LastSupposedPose.setValues(supposedPosition);

        double errorX, errorY, errorRotZ;
        RobotPose2D offsetSinceStart = relativePosition;

        RobotPose2D error = XYPlaneCalculations.getRelativePosition(offsetSinceStart,supposedPosition);

        this.m_LastError = error;

        errorX = error.X;
        errorY = error.Y;
        errorRotZ = error.getRotationZ();

        this.m_MotionSystem.getPIDCalculator().feedError(errorX,errorY,errorRotZ);
        RobotVector2D correctionVelocity = this.m_MotionSystem.getPIDCalculator().getPIDPower();

        return correctionVelocity;
    }

    protected void updateSupposedPos(RobotPose2D supposedPose){
        this.m_LastSupposedPose = supposedPose;
    }

    protected void updateError(RobotPose2D error){
        this.m_LastError = error;
    }

    protected RobotVector2D setRobotSpeed(RobotVector2D robotSpeed, RobotPose2D supposedRelativePose){
        RobotVector2D correctionVector = this.getErrorCorrectionVelocityVector(supposedRelativePose);
        RobotVector2D afterCorrectionVector = new RobotVector2D(robotSpeed.X + correctionVector.X,robotSpeed.Y + correctionVector.Y,robotSpeed.getRotationZ() + correctionVector.getRotationZ());
        RobotVector2D afterCorrectionTheoraticalMaximum = this.m_MotionSystem.getTheoreticalMaximumMotionState(afterCorrectionVector);
        RobotVector2D actualVector = null;
        RobotVector2D returnVector = null;
        if(Math.abs(afterCorrectionVector.X) > Math.abs(afterCorrectionTheoraticalMaximum.X) || Math.abs(afterCorrectionVector.Y) > Math.abs(afterCorrectionTheoraticalMaximum.Y) || Math.abs(afterCorrectionVector.getRotationZ()) > Math.abs(afterCorrectionTheoraticalMaximum.getRotationZ())){
            actualVector = afterCorrectionTheoraticalMaximum;
            double factor = actualVector.X / afterCorrectionVector.X;
            returnVector = new RobotVector2D(robotSpeed.X * factor,robotSpeed.Y * factor, robotSpeed.getRotationZ() * factor);
        }else{
            actualVector = afterCorrectionVector;
            returnVector = new RobotVector2D(robotSpeed);
        }
        this.getMotionSystem().__setRobotSpeed(actualVector.X,actualVector.Y,actualVector.getRotationZ());
        return returnVector;
    }
  
    public RobotPose2D getLastSupposedPose(){
        return this.m_LastSupposedPose;
    }
  
    public RobotPose2D getLastError(){
        return this.m_LastError;
    }

    public abstract void drawPath(Canvas dashboardCanvas);
}
