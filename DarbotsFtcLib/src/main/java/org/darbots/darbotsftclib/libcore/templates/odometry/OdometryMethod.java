package org.darbots.darbotsftclib.libcore.templates.odometry;

public abstract class OdometryMethod {
    private CustomizableOdometry m_OdometerContainer = null;
    public OdometryMethod(){
        this.m_OdometerContainer = null;
    }
    public OdometryMethod(OdometryMethod oldMethod){
        this.m_OdometerContainer = oldMethod.m_OdometerContainer;
    }
    public abstract void __trackStart();
    public abstract void __trackLoop(double secondsSinceLastLoop);
    public abstract void __trackLoop_NoActualPositionShift(double secondsSinceLastLoop);
    public CustomizableOdometry getPositionTracker(){
        return this.m_OdometerContainer;
    }
    public void setPositionTracker(CustomizableOdometry container){
        this.m_OdometerContainer = container;
    }
}
