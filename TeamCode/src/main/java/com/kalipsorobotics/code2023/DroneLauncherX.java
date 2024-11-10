package com.kalipsorobotics.code2023;

import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.concurrent.locks.ReentrantLock;

class DroneLauncherX {
    private final DroneLauncherMotor droneLauncherMotor;
    private final static double TARGET_VELOCITY = ((0.028 * 100) * 0.5);
    final private DroneLauncherServo droneLauncherServo;
    private final ReentrantLock lock = new ReentrantLock();


    private DroneLauncherX(DroneLauncherServo droneLauncherServo, DroneLauncherMotor droneLauncherMotor) {
        this.droneLauncherMotor = droneLauncherMotor;
        this.droneLauncherServo = droneLauncherServo;
    }

    public DroneLauncherX(OpModeUtilities opModeUtilities) {
        this.droneLauncherMotor = new DroneLauncherMotor(opModeUtilities);
        this.droneLauncherServo = new DroneLauncherServo(opModeUtilities);
    }

    public void launch() throws InterruptedException {
        lock.lock();
        try {
            double power = 0.2;
            droneLauncherServo.disEngage();
            while (true) {
                droneLauncherMotor.setPower(power);

                int planeLauncherTicksBeforeSleep = droneLauncherMotor.getPosTicks();

                Thread.sleep(100);

                int planeLauncherTicksAfterSleep = droneLauncherMotor.getPosTicks();
                planeLauncherTicksAfterSleep = planeLauncherTicksBeforeSleep - planeLauncherTicksAfterSleep;
                double currentVelocity = Math.abs(planeLauncherTicksAfterSleep / 100.);


                if ((currentVelocity >= TARGET_VELOCITY) || (power >= 1)) {
                    //launch
                    droneLauncherServo.engage();
                    // Log.d("drone", "reach velocity: " + (currentVelocity >= targetVelocity));
                    Thread.sleep(1000);
                    break;
                } else {
                    power += 0.05;

                    // Log.d("drone", "current velocity: " + currentVelocity + "target velocity: " + targetVelocity + "current power" + initialPlaneLauncherPower);
                    // telemetry.addData("currentVelocity: ", currentVelocity);
                    // telemetry.addData("targetVelocity: ", targetVelocity);
                }
            }
            droneLauncherMotor.off();
        } finally {
            lock.unlock();
        }
    }

    public void readyLauncher() throws InterruptedException {
        droneLauncherServo.disEngage();
    }

}
