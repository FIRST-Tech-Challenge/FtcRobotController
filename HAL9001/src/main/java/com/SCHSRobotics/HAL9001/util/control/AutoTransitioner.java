package com.SCHSRobotics.HAL9001.util.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by KNO3 Robotics
 * AutoTransitioner is a utility to automatically initialize the teleop program of your choice
 * after the autonomous period ends. To use AutoTransitioner, go to your OpMode/LinearOpMode,
 * and place the following line of code in your init() method or before your waitForStart()
 * (for OpMode and LinearOpMode, respectively):
 * AutoTransitioner.transitionOnStop(this, "MainRobot Teleop");
 * Where 'MainRobot Teleop' is replaced with the NAME of your teleop program. See full documentation
 * on kno3.net/resources for more info.
 *
 * @see com.SCHSRobotics.HAL9001.system.robot.LinkTo
 */
public class AutoTransitioner extends Thread {
    private static final AutoTransitioner INSTANCE = new AutoTransitioner(); //Create singleton instance

    private OpMode onStop;
    private String transitionTo;
    private OpModeManagerImpl opModeManager;

    private AutoTransitioner() {
        this.start(); //Start the watcher thread
    }

    @Override
    @SuppressWarnings({"InfiniteLoopStatement","EmptyCatchBlock"})
    public void run() {
        try {
            while (true) { //Loop
                synchronized (this) { //Synchronized to prevent weird conditions
                    //If there is a transition set up and the active op mode is no longer the one
                    //the transition was set up with, proceed with the transition
                    if (onStop != null && opModeManager.getActiveOpMode() != onStop) {
                        Thread.sleep(1000); //Wait 1 second to prevent weird conditions
                        opModeManager.initActiveOpMode(transitionTo); //Request initialization of the teleop
                        reset(); //Reset the AutoTransitioner
                    }
                }
                Thread.sleep(50); //Sleep 50 seconds to minimize performance impact to the rest of your program
            }
        } catch (InterruptedException ex) {
        }
    }

    private void setNewTransition(OpMode onStop, String transitionTo) {
        synchronized (this) { //Synchronized to prevent weird conditions
            this.onStop = onStop;
            this.transitionTo = transitionTo;
            this.opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices; //Store OpModeManagerImpl
        }
    }

    private void reset() {
        this.onStop = null;
        this.transitionTo = null;
        this.opModeManager = null;
    }

    /**
     * Setup the next transition
     * @param onStop The program you'll be transitioning from (usually 'this')
     * @param transitionTo The name of the program you want to transition to
     */
    public static void transitionOnStop(OpMode onStop, String transitionTo) {
        INSTANCE.setNewTransition(onStop, transitionTo);
    }
}
