package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.DriverRumble.Output;
import org.junit.Test;

public class DriverRumbleTest {
    @Test public void queueFullPulsesOncePerFillAndTheFirstReadingIsABaseline() {
        DriverRumble r = new DriverRumble();
        assertNull(r.next(0, true, false, false)); // Already full at START: baseline, no cue.
        assertNull(r.next(1, false, false, false));
        assertEquals(Output.QUEUE_FULL, r.next(2, true, false, false));
        assertEquals(Output.OFF, r.next(2 + DriverRumble.QUEUE_FULL_CUE_SECONDS, true, false, false));
        assertNull(r.next(3, true, false, false)); // Held full: no repeat.
        r.next(4, false, false, false);
        assertEquals(Output.QUEUE_FULL, r.next(5, true, false, false));
    }

    @Test public void holdingRtRumblesUntilTheShotIsReady() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, false, false);
        assertEquals(Output.AIMING, r.next(.1, false, true, false));
        assertNull(r.next(.2, false, true, false)); // Commanded once, not every loop.
        assertEquals(Output.OFF, r.next(.3, false, true, true));
        assertNull(r.next(.4, false, true, true));
        assertNull(r.next(1, false, false, false)); // RT released: already off.
        assertEquals(Output.AIMING, r.next(1.1, false, true, false));
        // RT released before the shot was ready: the rumble stops.
        assertEquals(Output.OFF, r.next(1.2, false, false, false));
    }

    @Test public void lostReadinessResumesTheRumbleOnlyAfterTheResumeDelay() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, true, false);
        assertEquals(Output.OFF, r.next(.1, false, true, true));
        assertNull(r.next(.2, false, true, false)); // Flicker: still off.
        assertNull(r.next(.3, false, true, true));
        assertNull(r.next(.4, false, true, false));
        assertEquals(Output.AIMING, r.next(.4 + DriverRumble.NOT_READY_RESUME_SECONDS + .01, false, true, false));
        // A new RT press rumbles immediately again.
        assertEquals(Output.OFF, r.next(1, false, false, false));
        assertEquals(Output.AIMING, r.next(1.1, false, true, false));
    }

    @Test public void queueFullPulsesPlayOutThenTheAimingRumbleResumes() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, false, false);
        assertEquals(Output.AIMING, r.next(.1, false, true, false));
        assertEquals(Output.QUEUE_FULL, r.next(.2, true, true, false));
        assertNull(r.next(.3, true, true, false)); // Pulses still playing.
        assertEquals(Output.AIMING, r.next(.2 + DriverRumble.QUEUE_FULL_CUE_SECONDS + .01, true, true, false));
    }

    @Test public void updateCommandsTheSdkGamepad() {
        DriverRumble r = new DriverRumble();
        Gamepad gp1 = new Gamepad();
        r.update(0, false, true, false, gp1);
        assertTrue(gp1.isRumbling()); // Continuous until stopped.
        Gamepad.RumbleEffect aiming = gp1.rumbleQueue.peek();
        assertEquals(1, aiming.steps.size());
        assertEquals(Gamepad.RUMBLE_DURATION_CONTINUOUS, aiming.steps.get(0).duration);
        assertEquals(102, aiming.steps.get(0).large); // 0.4 power.
        r.update(.1, false, true, true, gp1);
        assertFalse(gp1.isRumbling());
        r.update(.2, true, true, true, gp1);
        Gamepad.RumbleEffect full = gp1.rumbleQueue.peek();
        assertEquals(3, full.steps.size()); // Pulse, gap, pulse.
        assertEquals(255, full.steps.get(0).large);
        DriverRumble.stop(gp1, null);
        assertFalse(gp1.isRumbling());
    }
}
