package tests;

import org.junit.jupiter.api.Test;
import org.firstinspires.ftc.teamcode.util.*;
import static org.junit.jupiter.api.Assertions.*;

public class motorRampProfileTests {
    DummyElapsedTime det;
    motorRampProfile mrp;
    public motorRampProfileTests() {
        det = new DummyElapsedTime(0);
        mrp = new motorRampProfile(1.0,det);
    }

    @Test
    public void checkSetRampRate() {
        motorRampProfile mrp = new motorRampProfile(1.0);
        final double newvalue=2.0;
        mrp.setRampRate(newvalue);
        assertEquals(newvalue, mrp.getRampRate());
    }

    @Test
    void rampRateTest1second() {
        det.setFakeSeconds(1);
        double result = mrp.ramp(1.0);
        assertEquals(1,result);
    }

    @Test
    void rampRateTest10seconds() {
        det.setFakeSeconds(1);
        double result = mrp.ramp(2.0);
        det.setFakeSeconds(2);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(3);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(4);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(5);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(6);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(7);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(8);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(9);
        result = mrp.ramp(2.0);
        det.setFakeSeconds(10);
        result = mrp.ramp(2.0);

        assertEquals(2,result);

    }

}
