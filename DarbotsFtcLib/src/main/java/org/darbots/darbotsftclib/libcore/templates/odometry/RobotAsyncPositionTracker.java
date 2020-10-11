package org.darbots.darbotsftclib.libcore.templates.odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class RobotAsyncPositionTracker extends RobotBasic2DPositionTracker implements CustomizableOdometry, RobotNonBlockingDevice {
    private RobotGyro m_GyroProvider = null;
    private float m_LastGyroReading;
    private float m_GyroReadingAtZero;
    private OdometryMethod m_Method;
    protected volatile double m_XDistanceFactor, m_YDistanceFactor, m_ZRotDistanceFactor;
    private boolean m_IsBusy = false;
    private ElapsedTime m_Time;

    public RobotAsyncPositionTracker(OdometryMethod odometryMethod, RobotPose2D initialPosition) {
        super(initialPosition);
        this.m_XDistanceFactor = 1;
        this.m_YDistanceFactor = 1;
        this.m_ZRotDistanceFactor = 1;
        this.m_Method = odometryMethod;
        this.__setupAsyncTracking();
    }

    public RobotAsyncPositionTracker(RobotAsyncPositionTracker oldTracker){
        super(oldTracker);
        RobotVector2D oldDistanceFactor = oldTracker.getDistanceFactors();
        this.m_XDistanceFactor = oldDistanceFactor.X;
        this.m_YDistanceFactor = oldDistanceFactor.Y;
        this.m_ZRotDistanceFactor = oldDistanceFactor.getRotationZ();
        this.setGyroProvider(oldTracker.m_GyroProvider);
        this.m_Method = oldTracker.m_Method;
        this.__setupAsyncTracking();
    }

    protected void __setupAsyncTracking(){
        this.m_Method.setPositionTracker(this);
    }

    @Override
    public RobotVector2D getDistanceFactors(){
        return new RobotVector2D(this.m_XDistanceFactor,this.m_YDistanceFactor,this.m_ZRotDistanceFactor);
    }

    @Override
    public double getXDistanceFactor(){
        return this.m_XDistanceFactor;
    }

    @Override
    public double getYDistanceFactor(){
        return this.m_YDistanceFactor;
    }

    @Override
    public double getRotZDistanceFactor(){
        return this.m_ZRotDistanceFactor;
    }

    /**
     * Set X,Y,ZRot Distance Factors to the Active Position Tracker
     * deltaX,Y,Z = trackedDeltaX,Y,Z / distanceFactor
     * @param DistanceFactor The distance factors of each axis / rotation
     */
    public void setDistanceFactors(RobotVector2D DistanceFactor){
        this.m_XDistanceFactor = DistanceFactor.X;
        this.m_YDistanceFactor = DistanceFactor.Y;
        this.m_ZRotDistanceFactor = DistanceFactor.getRotationZ();
    }

    protected void drive_MoveThroughRobotAxisOffset(RobotPose2D robotAxisValues) {
        RobotPose2D currentPose = super.m_CurrentPos;
        RobotPose2D tempField = XYPlaneCalculations.getAbsolutePosition(currentPose,robotAxisValues);
        super.m_CurrentPos.X = tempField.X;
        super.m_CurrentPos.Y = tempField.Y;
        super.m_CurrentPos.setRotationZ(tempField.getRotationZ());
    }

    @Override
    public void __trackLoopMoved(RobotVector2D velocity, RobotPose2D deltaRobotAxis){
        RobotVector2D fixedVelocity = new RobotVector2D(velocity.X / this.m_XDistanceFactor,velocity.Y / this.m_YDistanceFactor,velocity.getRotationZ() / this.m_ZRotDistanceFactor);
        RobotPose2D fixedDeltaRobotAxis = new RobotPose2D(deltaRobotAxis.X / this.m_XDistanceFactor, deltaRobotAxis.Y / this.m_YDistanceFactor, deltaRobotAxis.getRotationZ() / this.m_ZRotDistanceFactor);
        this.__trackLoopMovedRaw(fixedVelocity,fixedDeltaRobotAxis);
    }

    @Override
    public void __trackLoopMovedRaw(RobotVector2D velocity, RobotPose2D deltaRobotAxis){
        this.setCurrentVelocityVector(velocity);
        this.drive_MoveThroughRobotAxisOffset(deltaRobotAxis);
    }

    @Override
    public boolean isBusy() {
        return true;
    }

    protected void __trackStart(){
        this.m_Method.__trackStart();
    }
    protected void __trackLoop(double secondsSinceLastLoop){
        this.m_Method.__trackLoop(secondsSinceLastLoop);
    }

    public void start(){
        if(m_IsBusy){
            return;
        }
        m_IsBusy = true;
        this.m_Method.__trackStart();
        this.m_Time = new ElapsedTime();
    }

    @Override
    public void updateStatus() {
        if(this.m_IsBusy){
            double timeSince = this.m_Time.seconds();
            this.m_Time.reset();
            this.__trackLoop(timeSince);
        }
    }

    @Override
    public void waitUntilFinish() {
        return;
    }

    @Override
    public void setGyroProvider(RobotGyro provider){
        this.m_GyroProvider = provider;
        if(provider != null) {
            this.updateGyroProvider();
            this.m_LastGyroReading = this.m_GyroProvider.getHeading();
            RobotPose2D currentPosition = super.getCurrentPosition();
            this.m_GyroReadingAtZero = this.m_LastGyroReading - ((float) currentPosition.getRotationZ());
        }
    }

    @Override
    public void updateGyroProvider(){
        if(this.m_GyroProvider != null && this.m_GyroProvider instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) this.m_GyroProvider).updateStatus();
        }
    }

    @Override
    public double __getDeltaAng(double supposedDeltaAng){
        if(this.m_GyroProvider == null){
            return supposedDeltaAng;
        }else{
            this.updateGyroProvider();
            float newAng = this.m_GyroProvider.getHeading();
            float deltaAngGyro = XYPlaneCalculations.normalizeDeg(newAng - this.m_LastGyroReading);
            if(this.m_GyroProvider.getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise){
                deltaAngGyro = -deltaAngGyro;
            }
            this.m_LastGyroReading = newAng;
            return deltaAngGyro;
        }
    }

    @Override
    public RobotPose2D getCurrentPosition(){
        RobotPose2D currentPose = super.getCurrentPosition();
        if(this.m_GyroProvider != null){
            double currentGyroReading = this.m_GyroProvider.getHeading();
            double deltaAng = currentGyroReading - this.m_GyroReadingAtZero;
            if(this.m_GyroProvider.getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise){
                deltaAng = -deltaAng;
            }
            currentPose.setRotationZ(deltaAng);
        }
        return currentPose;
    }

    @Override
    public void setCurrentPosition(RobotPose2D currentPosition){
        super.setCurrentPosition(currentPosition);
        if(this.m_GyroProvider != null) {
            this.updateGyroProvider();
            this.m_LastGyroReading = this.m_GyroProvider.getHeading();
            this.m_GyroReadingAtZero = XYPlaneCalculations.normalizeDeg(this.m_LastGyroReading - ((float) currentPosition.getRotationZ()));
        }
    }

    @Override
    public void stop() {
        if(!this.m_IsBusy){
            return;
        }
        this.m_IsBusy = false;
        this.m_Time = null;
    }
}
