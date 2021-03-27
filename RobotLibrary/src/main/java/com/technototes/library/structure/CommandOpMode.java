package com.technototes.library.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.hardware.HardwareDevice;
import com.technototes.logger.Logger;

/** Class for command based op modes
 * @author Alex Stedman
 */
public abstract class CommandOpMode extends LinearOpMode {
    /*** Command gamepad object
     *
     */
    public CommandGamepad driverGamepad, codriverGamepad;

    private ElapsedTime opModeTimer = new ElapsedTime();

    private OpModeState opModeState = OpModeState.INIT;

    private Logger logger;

    /** Get op mode state
     *
     * @return Current op mode state
     */
    public OpModeState getOpModeState() {
        return opModeState;
    }

    /** Get the op mode's logger
     *
     * @return The logger
     */
    public Logger getLogger() {
        return logger;
    }

    /** Get the opmode runtime
     *
     * @return Runtime in seconds
     */
    public double getOpModeRuntime(){
        return opModeTimer.seconds();
    }

    @Override
    public final void runOpMode() {
        driverGamepad = new CommandGamepad(gamepad1);
        codriverGamepad = new CommandGamepad(gamepad2);
        HardwareDevice.hardwareMap = hardwareMap;
        CommandScheduler.getInstance().opMode = this;
        opModeTimer.reset();
        logger = new Logger(this);
        uponInit();
        while (!isStarted()) {
            initLoop();
            universalLoop();
            CommandScheduler.getInstance().run();
            logger.initUpdate();
        }
        opModeState = OpModeState.RUN;
        uponStart();
        while (opModeIsActive()) {
            runLoop();
            universalLoop();
            CommandScheduler.getInstance().run();
            logger.runUpdate();
            driverGamepad.periodic();
            codriverGamepad.periodic();
        }
        opModeState = OpModeState.END;
        end();
        CommandScheduler.getInstance().run();
        opModeTimer.reset();

    }

    /** Runs once when op mode is initialized
     *
     */
    public void uponInit() {

    }
    /** Runs constantly when op mode is initialized, yet not started
     *
     */
    public void initLoop() {

    }

    /** Runs once when op mode is started
     *
     */
    public void uponStart() {

    }
    /** Runs constantly when op mode is started
     *
     */
    public void runLoop() {

    }
    /** Runs once when op mode is ended
     *
     */
    public void end() {

    }

    /** Runs constantly during all periods
     *
     */
    public void universalLoop(){

    }

    /** Enum for op mode state
     *
     */
    public enum OpModeState {
        INIT, RUN, END;

        /** Check if other states are this state
         *
         * @param states The other states
         * @return The above condition
         */
        public boolean isState(OpModeState... states){
            for(OpModeState s : states){
                if(this == s){
                    return true;
                }
            }
            return false;
        }
    }
}
