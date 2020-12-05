package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.Robot;

public class MecanumRunnableLQR implements Runnable {
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private boolean readyToUpdate;
    private boolean stop;

    private MecanumDriveMPC lqrDrivetrain;
    private SimpleMatrix desiredState;
    private double policyLag;

    public MecanumRunnableLQR() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
        setDesiredState(null);
    }

    public MecanumDriveMPC lqr(SimpleMatrix desiredState) {
        MecanumDriveMPC mpc = new MecanumDriveMPC(true);
        mpc.model = Robot.getDriveModel();
        mpc.runLQR(desiredState);
        return mpc;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            if(!isReadyToUpdate()) {
                getPolicyTimeProfiler().start();
                setLqrDrivetrain(lqr(getDesiredState()));
                getTimeProfiler().update(true);
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }

                setReadyToUpdate(true);
            }

            try {
                Thread.sleep(1);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void updateMPC() {
        if(isReadyToUpdate() && getLqrDrivetrain() != null) {
            Robot.setDriveMPC(getLqrDrivetrain());
            setPolicyLag(getPolicyTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
            setReadyToUpdate(false);
        }
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public TimeProfiler getPolicyTimeProfiler() {
        return policyTimeProfiler;
    }

    public void setPolicyTimeProfiler(TimeProfiler policyTimeProfiler) {
        this.policyTimeProfiler = policyTimeProfiler;
    }

    public boolean isReadyToUpdate() {
        return readyToUpdate;
    }

    public void setReadyToUpdate(boolean readyToUpdate) {
        this.readyToUpdate = readyToUpdate;
    }

    public boolean isStop() {
        return stop;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }

    public MecanumDriveMPC getLqrDrivetrain() {
        return lqrDrivetrain;
    }

    public void setLqrDrivetrain(MecanumDriveMPC lqrDrivetrain) {
        this.lqrDrivetrain = lqrDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }
}
