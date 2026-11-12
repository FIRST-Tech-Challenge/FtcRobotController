package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.Arrays;
import java.util.Collections;
import static org.junit.Assert.*;

public class PollenConfigurationTest {
    private PollenTracker.Detection blob(double x, double area, boolean clipped) {
        return new PollenTracker.Detection(x,180,x,200,area,.8,1,.8,clipped);
    }
    @Test public void measuredZeroOffsetsStillRequireOtherCalibration() {
        assertFalse(PollenConfiguration.diagnostic().autonomousEligible(true));
        PollenConfiguration c = new PollenConfiguration(180,15,0,0,60,0,0,0,0,true,false,true,true,false,20,40,80,80,350,200);
        assertFalse(c.autonomousEligible(false)); assertTrue(c.autonomousEligible(true));
        assertTrue(c.status(false).contains("Approximate"));
        c = new PollenConfiguration(180,15,0,0,60,200,200,160,120,true,true,true,true,false,20,40,80,80,350,200);
        assertTrue(c.autonomousEligible(false));
    }
    @Test public void invalidCalibrationIsRejected() {
        assertThrows(IllegalArgumentException.class,()->new PollenConfiguration(0,15,0,0,60,0,0,0,0,false,false,false,false,false,20,40,80,80,350,200));
        assertThrows(IllegalArgumentException.class,()->new PollenConfiguration(180,15,0,0,60,0,0,0,0,true,true,true,true,false,20,40,80,80,350,200));
        assertThrows(IllegalArgumentException.class,()->new PollenConfiguration(180,15,0,0,60,200,200,160,120,true,true,true,true,false,80,40,80,80,350,200));
    }
    @Test public void clippingFreshnessAndContinuityUseActualFrameTimes() {
        PollenTracker t = new PollenTracker(PollenConfiguration.diagnostic());
        assertNull(t.select(1000000000L,Collections.singletonList(blob(160,1000,true)),200,200,160,120));
        assertTrue(t.rejection().contains("Clipped"));
        PollenTarget a=t.select(1010000000L,Collections.singletonList(blob(160,1000,false)),200,200,160,120);
        assertNotNull(a);
        PollenTarget b=t.select(1020000000L,Arrays.asList(blob(165,100,false),blob(250,2000,false)),200,200,160,120);
        assertEquals(165,b.centerXPx,0);
        assertNull(t.select(1030000000L,Collections.singletonList(blob(250,2000,false)),200,200,160,120));
        assertNotNull(t.select(1600000000L,Collections.singletonList(blob(250,2000,false)),200,200,160,120));
        assertTrue(PollenTracker.fresh(a.captureTimeNanos,1100000000L));
        assertFalse(PollenTracker.fresh(a.captureTimeNanos,1400000000L));
        assertFalse(PollenTracker.fresh(a.captureTimeNanos,1000000000L));
    }
    @Test public void floorContactOffsetTransformAndInvalidRay() {
        PollenTarget a=PollenTarget.fromFloorContact(1,160,160,160,200,1000,200,200,160,120,180,15,0,0);
        PollenTarget b=PollenTarget.fromFloorContact(1,160,160,160,200,1000,200,200,160,120,180,15,50,-20);
        assertNotNull(a); assertEquals(a.forwardMm+50,b.forwardMm,1e-8); assertEquals(-20,b.rightMm,1e-8);
        assertEquals(200,b.contactYPx,0);
        assertNull(PollenTarget.fromFloorContact(1,160,160,Double.NaN,200,1000,200,200,160,120,180,15,0,0));
    }
}
