package org.firstinspires.ftc.teamcode.support;

/**
 * Created by 28761 on 9/4/2018.
 */

public class CoreSystem {

    private YieldHandler yieldHandler = DEFAULT_HANDLER;

    public CoreSystem() {

    }

    final static YieldHandler DEFAULT_HANDLER = new YieldHandler() {
        @Override
        public void on_yield() {
            // Default handler, does nothing
        }
    };

    public void set_yield_handler(YieldHandler y) {
        yieldHandler = y;
    }

    public void yield() {
        yieldHandler.on_yield();
    }

    // TODO rename to yieldFor
    public void yield_for(double seconds) {
        long millisec = Math.round(seconds * 1000);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < millisec)
            yield();
    }
}