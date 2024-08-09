package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.NewStuff.PIDController;
import org.firstinspires.ftc.teamcode.NewStuff.PIDState;
import org.junit.Test;

public class PIDControllerTest {
    @Test
    public void testCalculatePIDNonMecanum() {
        // at initial position
        final PIDState state = new PIDState(0.005, 0.0000005, 0.4, 0.0, false);
        double power = PIDController.calculatePID(0, 3000, 1e-9, state);
        assertTrue(power > 0);
        assertEquals(0, state.integral, 0);
        assertEquals(0, state.derivative, 0);

        // somewhere on the path
        state.lastTime = 100.0;
        power = PIDController.calculatePID(1000, 3000, 200.0, state);
        assertTrue(power > 0);
//        assertTrue(state.integral > 0);
//        assertTrue(state.derivative > 0);

        // close
        power = PIDController.calculatePID(3000, 3000, 400.0, state);
        assertTrue(power<0);

        // very close
        power = PIDController.calculatePID(2990, 3000, 500.0, state);
        assertEquals(0, power, 0.2);

        // arrived
        power = PIDController.calculatePID(3000, 3000, 600.0, state);
        assertEquals(0, power, 0.1);
//        assertTrue(state.integral > 0);
//        assertTrue(state.derivative > 0);
    }
}