package org.firstinspires.ftc.teamcode.opmodes.mechBot; //set your package

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.opmodes.mechBot.TeleDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public abstract class TeleDrive extends OpMode {
    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    private TeleDrive.LinearOpModeHelper helper          = null;
    private ExecutorService executorService = null;
    private volatile boolean   isStarted       = false;
    private volatile boolean   stopRequested   = false;
    private boolean            userMonitoredForStart = false;
    private final Object       runningNotifier = new Object();


    private DatagramSocket socket;
    private boolean canRunGamepadThread;
    private Thread gamepadHandler;

    private void startGamepadHandlerThread() {
        telemetry.setAutoClear(true);
        gamepadHandler = new Thread(new Runnable() {
            @Override
            public void run() {
                while (canRunGamepadThread) {
                    String gamepadAction = "";
                    try {
                        byte[] buffer = new byte[1024];
                        DatagramPacket response = new DatagramPacket(buffer, buffer.length);
                        socket.receive(response);
                        gamepadAction = new String(buffer, 0, response.getLength());
                    } catch (Exception ignore) {

                    }

                    if (!gamepadAction.isEmpty()) {
                        if (gamepadAction.contains("G1")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("UP")) {
                                    gamepad1.dpad_up = true;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DOWN")) {
                                    gamepad1.dpad_down = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("LEFT")) {
                                    gamepad1.dpad_left = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("RIGHT")) {
                                    gamepad1.dpad_right = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.back = false;
                                }
                            }
                            if (gamepadAction.contains("_Jx")) {
                                gamepad1.left_stick_x = Float.parseFloat(gamepadAction.replace("G1_Jx_", ""));
                            }
                            if (gamepadAction.contains("_Jy")) {
                                gamepad1.left_stick_y = Float.parseFloat(gamepadAction.replace("G1_Jy_", ""));
                            }
                            if (gamepadAction.contains("_Jz")) {
                                gamepad1.right_stick_x = Float.parseFloat(gamepadAction.replace("G1_Jz_", ""));
                            }
                            if (gamepadAction.contains("_Jrz")) {
                                gamepad1.right_stick_y = Float.parseFloat(gamepadAction.replace("G1_Jrz_", ""));
                            }
                        }
                        if (gamepadAction.contains("G2")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("UP")) {
                                    gamepad2.dpad_up = true;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("DOWN")) {
                                    gamepad2.dpad_down = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("LEFT")) {
                                    gamepad2.dpad_left = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("RIGHT")) {
                                    gamepad2.dpad_right = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.back = false;
                                }
                            }
                            if (gamepadAction.contains("_Jx")) {
                                gamepad2.left_stick_x = Float.parseFloat(gamepadAction.replace("G2_Jx_", ""));
                            }
                            if (gamepadAction.contains("_Jy")) {
                                gamepad2.left_stick_y = Float.parseFloat(gamepadAction.replace("G2_Jy_", ""));
                            }
                            if (gamepadAction.contains("_Jz")) {
                                gamepad2.right_stick_x = Float.parseFloat(gamepadAction.replace("G2_Jz_", ""));
                            }
                            if (gamepadAction.contains("_Jrz")) {
                                gamepad2.right_stick_y = Float.parseFloat(gamepadAction.replace("G2_Jrz_", ""));
                            }
                        }
                    }
                }
                gamepadHandler.interrupt();
            }
        });
        gamepadHandler.setName("Gamepad Handler Thread");
        gamepadHandler.setPriority(Thread.NORM_PRIORITY);
        gamepadHandler.start();
    }

    //------------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------------

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     * @throws InterruptedException
     */
    abstract public void runOpMode() throws InterruptedException;

    /**
     * Pauses the Linear Op Mode until start has been pressed or until the current thread
     * is interrupted.
     */
    public void waitForStart() {
        while (!isStarted()) {
            synchronized (runningNotifier) {
                try {
                    runningNotifier.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    /**
     * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
     * threads in the system to run.
     *
     * <p>One can use this method when you have nothing better to do in your code as you await state
     * managed by other threads to change. Calling idle() is entirely optional: it just helps make
     * the system a little more responsive and a little more efficient.</p>
     *
     * @see #opModeIsActive()
     */
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Answer as to whether this opMode is active and the robot should continue onwards. If the
     * opMode is not active, the OpMode should terminate at its earliest convenience.
     *
     * <p>Note that internally this method calls {@link #idle()}</p>
     *
     * @return whether the OpMode is currently active. If this returns false, you should
     *         break out of the loop in your {@link #runOpMode()} method and return to its caller.
     * @see #runOpMode()
     * @see #isStarted()
     * @see #isStopRequested()
     */
    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }

    /**
     * Has the opMode been started?
     *
     * @return whether this opMode has been started or not
     * @see #opModeIsActive()
     * @see #isStopRequested()
     */
    public final boolean isStarted() {

        /*
         * What we're looking for here is that the user polled until the
         * the start condition was occurred.
         */
        if(isStarted) userMonitoredForStart = true;

        return this.isStarted || Thread.currentThread().isInterrupted();
    }

    /**
     * Has the the stopping of the opMode been requested?
     *
     * @return whether stopping opMode has been requested or not
     * @see #opModeIsActive()
     * @see #isStarted()
     */
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void init() {
        this.executorService = ThreadPool.newSingleThreadExecutor("LinearOpMode");
        this.helper          = new TeleDrive.LinearOpModeHelper();
        this.isStarted       = false;
        this.stopRequested   = false;

        this.executorService.execute(helper);

        String address = "192.168.43.1";
        int port = 11039;
        canRunGamepadThread = false;

        try {
            this.socket = new DatagramSocket(port);
        } catch (Exception ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Connect your server to " + address + ":" + port, "");
        telemetry.update();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void init_loop() {
        handleLoop();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void start() {
        stopRequested = false;
        isStarted = true;
        synchronized (runningNotifier) {
            runningNotifier.notifyAll();
        }
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void loop() {
        handleLoop();
        if(!canRunGamepadThread) {
            canRunGamepadThread = true;
            startGamepadHandlerThread();
        }
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void stop() {

        /*
         * Get out of dodge. Been here, done this.
         * (If a linear opmode returns of its own accord, this is
         *  invoked twice).
         */
        if(stopRequested) return;

        /*
         * Handle edge case of stop() before init()
         */
        if(helper == null) return;

        /*
         * Is it ending because it simply... ended (e.g. end of auto), or
         * because the user failed to monitor for the start condition?
         *
         * We must check userMethodReturned, because if it didn't return,
         * but also !userMonitoredForStart, that means the opmode was aborted
         * during init. We don't want to show a warning in that case.
         */
        if(!userMonitoredForStart && helper.userMethodReturned) {
            RobotLog.addGlobalWarningMessage("The OpMode which was just initialized ended prematurely as a result of not monitoring for the start condition. Did you forget to call waitForStart()?");
        }

        // make isStopRequested() return true (and opModeIsActive() return false)
        stopRequested = true;

        if (executorService != null) {  // paranoia

            // interrupt the linear opMode and shutdown it's service thread
            executorService.shutdownNow();

            /** Wait, forever, for the OpMode to stop. If this takes too long, then
             * {@link OpModeManagerImpl#callActiveOpModeStop()} will catch that and take action */
            try {
                String serviceName = "user linear op mode";
                ThreadPool.awaitTermination(executorService, 100, TimeUnit.DAYS, serviceName);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        super.stop();
        canRunGamepadThread = false;
        socket.close();
    }

    protected void handleLoop() {
        // if there is a runtime exception in user code; throw it so the normal error
        // reporting process can handle it
        if (helper.hasRuntimeException()) {
            throw helper.getRuntimeException();
        }

        synchronized (runningNotifier) {
            runningNotifier.notifyAll();
        }
    }


    protected class LinearOpModeHelper implements Runnable {

        protected RuntimeException exception  = null;
        protected boolean          isShutdown = false;
        protected volatile boolean userMethodReturned = false;

        public LinearOpModeHelper() {
        }

        @Override
        public void run() {
            ThreadPool.logThreadLifeCycle("LinearOpMode main", new Runnable() { @Override public void run() {
                exception = null;
                isShutdown = false;

                try {
                    TeleDrive.this.runOpMode();
                    userMethodReturned = true;
                    requestOpModeStop();
                } catch (InterruptedException ie) {
                    // InterruptedException, shutting down the op mode
                    RobotLog.d("LinearOpMode received an InterruptedException; shutting down this linear op mode");
                } catch (CancellationException ie) {
                    // In our system, CancellationExceptions are thrown when data was trying to be acquired, but
                    // an interrupt occurred, and you're in the unfortunate situation that the data acquisition API
                    // involved doesn't allow InterruptedExceptions to be thrown. You can't return (what data would
                    // you return?), and so you have to throw a RuntimeException. CancellationException seems the
                    // best choice.
                    RobotLog.d("LinearOpMode received a CancellationException; shutting down this linear op mode");
                } catch (RuntimeException e) {
                    exception = e;
                } finally {
                    // If the user has given us a telemetry.update() that hasn't get gone out, then
                    // push it out now. However, any NEW device health warning should be suppressed while
                    // doing so, since required state might have been cleaned up by now and thus generate errors.
                    TimestampedI2cData.suppressNewHealthWarningsWhile(new Runnable() {
                        @Override public void run() {
                            if (telemetry instanceof TelemetryInternal) {
                                telemetry.setMsTransmissionInterval(0); // will be reset the next time the opmode runs
                                ((TelemetryInternal) telemetry).tryUpdateIfDirty();
                            }
                        }
                    });
                    // Do the necessary bookkeeping
                    isShutdown = true;
                }
            }});
        }

        public boolean hasRuntimeException() {
            return (exception != null);
        }

        public RuntimeException getRuntimeException() {
            return exception;
        }

        public boolean isShutdown() {
            return isShutdown;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry management
    //----------------------------------------------------------------------------------------------

    @Override public void internalPostInitLoop() {
        // Do NOT call super, as that updates telemetry unilaterally
        if (telemetry instanceof TelemetryInternal) {
            ((TelemetryInternal)telemetry).tryUpdateIfDirty();
        }
    }

    @Override public void internalPostLoop() {
        // Do NOT call super, as that updates telemetry unilaterally
        if (telemetry instanceof TelemetryInternal) {
            ((TelemetryInternal)telemetry).tryUpdateIfDirty();
        }
    }

}