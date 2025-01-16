package org.firstinspires.ftc.TBlib.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class TBVirtualSubsystem extends SubsystemBase {

    // When this is on the subsystem can't move. This is for situations when something goes crazy
    // for example the arm encoders disconnected and its doing weird shit, so we want to disable it to protect it.
    protected boolean m_isSubsystemDisabled = false;
    
    protected TBVirtualSubsystem(){
        init();
        initLogs();
    }
    // Do not implement this. If you to do something periodiclly implement updateValues()
    @Override
    public void periodic() {
        if (!m_isSubsystemDisabled) {
            updateValues();
        }
    }

    // Init everything
    protected void init() {}
    // Init only the logs
    protected void initLogs() {}

    protected void updateValues(){}

    public void disableSubsystem(){
        m_isSubsystemDisabled = true;
    }

    public void enableSubsystem(){
        m_isSubsystemDisabled = false;
    }
}
