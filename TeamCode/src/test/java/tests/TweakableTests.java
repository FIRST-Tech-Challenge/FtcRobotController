package tests;

import org.firstinspires.ftc.teamcode.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class TweakableTests {
    TweakableLong tLong;
    TweakableDouble tDouble;
    TweakableBoolean tBoolean;

    static long LongStart = 1000;
    static long LongAmount = 1;
    static double DoubleStart = 1000;
    static double DoubleAmount = .5;


    public TweakableTests() {
        tDouble = new TweakableDouble("TestDouble", DoubleAmount, DoubleStart);
        tBoolean = new TweakableBoolean("TestBoolean", false);
    }

    @Test
    public void TweakableLongIncrementTest() {
        tLong = new TweakableLong("TestLong", LongAmount, LongStart);
        tLong.adjustUp();
        assertEquals(LongStart + LongAmount,tLong.value);
        tLong=null;
    }

    @Test
    public void TweakableLongDecrementTest() {
        tLong = new TweakableLong("TestLong", LongAmount, LongStart);
        tLong.adjustDown();
        assertEquals(LongStart - LongAmount,tLong.value);
        tLong=null;
    }

    @Test
    public void TweakableDoubleIncrementTest() {
        tDouble = new TweakableDouble("TestDouble", DoubleAmount, DoubleStart);
        tDouble.adjustUp();
        assertEquals(DoubleStart+DoubleAmount,tDouble.value);
        tDouble=null;
    }

    @Test
    public void TweakableDoubleDecrementTest() {
        tDouble = new TweakableDouble("TestDouble", DoubleAmount, DoubleStart);
        tDouble.adjustDown();
        assertEquals(DoubleStart-DoubleAmount,tDouble.value);
        tDouble=null;
    }


    @Test
    public void TweakableBooleanToggleTests() {
        tBoolean = new TweakableBoolean("BooleanTest", false);
        assertFalse(tBoolean.value);
        tBoolean.adjustUp();
        assertTrue(tBoolean.value);
        tBoolean.adjustUp();
        assertFalse(tBoolean.value);
        tBoolean.adjustDown();
        assertTrue(tBoolean.value);
    }



}
