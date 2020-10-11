package org.darbots.darbotsftclib.libcore.templates;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;

/**
 * This class is used for programmers to
 */
public abstract class DarbotsAction implements RobotNonBlockingDevice {
    private boolean m_Busy = false;
    private boolean m_Finished = false;

    public DarbotsAction(){
        this.m_Busy = false;
        this.m_Finished = false;
    }
    public DarbotsAction(DarbotsAction action){
        this.m_Busy = false;
        this.m_Finished = false;
    }

    public void startAction(){
        if(this.m_Busy){
            return;
        }
        this.m_Busy = true;
        this.resetFinished();
        this.__startAction();
    }

    public boolean isActionFinished(){
        return this.m_Finished;
    }

    public void stopAction(){
        if(!this.m_Busy){
            return;
        }
        this.m_Busy = false;
        this.m_Finished = true;
        this.__stopAction();
    }

    public void resetFinished(){
        this.m_Finished = false;
    }

    @Override
    public boolean isBusy() {
        return this.m_Busy;
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }

    protected abstract void __startAction();
    protected abstract void __stopAction();
}
