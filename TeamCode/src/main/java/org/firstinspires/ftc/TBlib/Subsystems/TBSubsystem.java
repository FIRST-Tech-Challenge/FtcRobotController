package org.firstinspires.ftc.TBlib.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.TBlib.Subsystems.SubsystemManager;


public abstract class TBSubsystem<T, G> extends SubsystemBase {

    /** When this is on the subsystem is ready for calibration, staff in the dashboard... certain motors are disabled... etc */
    protected final boolean m_isInCalibrationMode;

    /** When this is true the subsystem can't activate motors. This is for situations when something goes crazy
    for example the arm encoders disconnected and its doing weird shit, so we want to disable it to protect it.*/
    protected boolean m_isSubsystemDisabled = false;
    
    /** The setpoint is what the subsystem is aiming for could be meters or degress or anything
    you want, and then it is translated to someting that the motors understand like Roatations or RPS. */
    protected T m_setpoint;
    /** The last setpoint of the subsystem.*/
    protected T m_lastSetpoint;
    
    /** The manager of the subsystem, used to run setpoint setting commands by an enum representing the desired state. */
    protected SubsystemManager<G> manager;

    /**
     *  The constractor for all the subsystems, runs initializing methods, is allways called from impementation.
     * @param isInCalibrationMode
     */
    protected TBSubsystem(boolean isInCalibrationMode){
        m_isInCalibrationMode = isInCalibrationMode;
        init();
    }
    
    // Do not implement this. If you to do something periodiclly implement updateValues()
    @Override
    public void periodic() {
        if (manager != null)
            manager.run();

        updateValues();
        if (!m_isInCalibrationMode && !m_isSubsystemDisabled && setpointChanged()) {
            applySetpoint();
        }
        if (manager != null)
            manager.run();
            
        if (m_isInCalibrationMode) 
            calibrateSubsystem();
    }

    /** Initialize every variable that doesn't depend on parameters from outside. */
    protected void init() {}


    /** Use this method to update every value in the subsystem and run the updateInputs method from  your IO class. */
    protected void updateValues(){}

    /**
     * 
     * @return the current setpoint the subsystem is aiming to.
     */
    public T getSetpoint(){
        return this.m_setpoint;
    }

    /**
     * Sets a new setpoint to aim to.
     * @param setpoint the desired setpoint.
     */
    public void setSetpoint(T setpoint){
        m_lastSetpoint = m_setpoint;
        this.m_setpoint = setpoint;
    }
    
    private boolean setpointChanged(){
        return m_setpoint.equals(m_lastSetpoint);
    }

    
    /**
     * 
     * @param tolerance the alowed error from the setpoint.
     * @return whether the subsystem has reached the setpoint or not.
     */
    public abstract boolean isAtSetpoint(T tolerance);
    
    /**
     * 
     * @return the current value of the subsystem in the same measurment as the setpoint, 
     * (e.g the current height of an elevator or the current RPM of a flywheel)
     */
    public abstract T getCurrentValue();

    /** 
     * This takes the setpoint and apply it to the IO
     */
    protected abstract void applySetpoint();

    /**
     * This should contain everything needed for calibrating the robot, you call this function 
     * and you can start calibrating.
     */
    protected void calibrateSubsystem(){};

    /**
     * Stops all the motors the subsystem controlls.
     */
    public abstract void stopMotors();

    /**
     * Stops all the motors and sets the {@link #m_isSubsystemDisabled} boolean to true.
     */
    public void disableSubsystem(){
        m_isSubsystemDisabled = true;
        stopMotors();
    }

    /**
     * Sets the {@link #m_isSubsystemDisabled} boolean to false.
     */
    public void enableSubsystem(){
        m_isSubsystemDisabled = false;
    }

    /**
     * Stops the override mode in the {@link #manager} to reenable requesting states.
     */
    public void stopOverride(){
        manager.stopOverride();
    }
}
