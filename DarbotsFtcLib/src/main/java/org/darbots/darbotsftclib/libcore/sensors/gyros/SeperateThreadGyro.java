package org.darbots.darbotsftclib.libcore.sensors.gyros;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;

public class SeperateThreadGyro implements GyroContainer {
    private class SeperateThreadGyroRunnable implements Runnable{
        private volatile boolean m_RunningCommand = false;
        private volatile boolean m_RunningFlag = false;

        @Override
        public void run() {
            m_RunningFlag = true;

            m_Method.initGyro();

            while(this.m_RunningCommand){
                try{
                    Thread.sleep(m_ThreadSleepTimeInMs);
                }catch(InterruptedException e){
                    e.printStackTrace();
                }
                m_Method.updateData();
                if(GlobalRegister.runningOpMode != null && GlobalRegister.runningOpMode.isStopRequested()){
                    break;
                }
            }
            m_RunningFlag = false;
            m_RunningCommand = false;
        }

        public void stop(){
            this.m_RunningCommand = false;
            while(this.m_RunningFlag){
                try {
                    Thread.sleep(20);
                }catch(InterruptedException e){

                }
            }
        }

        public boolean isRunning(){
            return this.m_RunningCommand || this.m_RunningFlag;
        }
    }

    private SeperateThreadGyroRunnable m_Runnable = null;
    private Thread m_GyroThread = null;
    private GyroMethod m_Method = null;
    protected volatile int m_ThreadSleepTimeInMs = 20;
    private volatile float m_Angle = 0.0f;
    public SeperateThreadGyro(GyroMethod method) {
        this.m_Method = method;
        this.__setupRunnableTracking();
    }

    public SeperateThreadGyro(SeperateThreadGyro oldGyro){
        this.m_Method = oldGyro.m_Method;
        this.__setupRunnableTracking();
    }

    protected void __setupRunnableTracking(){
        this.m_Method.setGyroContainer(this);
        this.m_Runnable = new SeperateThreadGyroRunnable();
    }

    @Override
    public float getHeading() {
        return this.m_Angle;
    }

    @Override
    public void setHeading(float Heading) {
        this.m_Angle = XYPlaneCalculations.normalizeDeg(Heading);
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return m_Method.getHeadingRotationPositiveOrientation();
    }

    public void terminate(){
        this.m_Runnable.stop();
    }

    public void start(){
        if(this.m_Runnable.isRunning()){
            return;
        }
        this.m_GyroThread = new Thread(this.m_Runnable);
        this.m_Runnable.m_RunningCommand = true;
        this.m_GyroThread.start();
    }
}
