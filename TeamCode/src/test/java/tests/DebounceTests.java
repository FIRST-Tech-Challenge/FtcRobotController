package tests;

import org.firstinspires.ftc.teamcode.util.Debounce;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;


public class DebounceTests {

    Debounce deb;
    long delayms = 1*1000; // 1 second debounce.
    public DebounceTests() {
        this.deb = new Debounce(delayms);
    }

    @Test
    public void checkPressFalseTest() {
        assertFalse(deb.checkPress(false));
    }

    @Test
    public void checkPressTrue() {
        assertTrue(deb.checkPress(true));
    }

    @Test
    public void checkPressFalseAfterOneSecond() {
        deb.checkPress(true);
        long curr = System.currentTimeMillis();
        while (System.currentTimeMillis() - curr < 1200) {
            // do nothing
        }
        assertFalse(deb.checkPress(false));
    }



    public void checkPressTrueAfterOneSecond() {
        deb.checkPress(true);
        long curr = System.currentTimeMillis();
        while (System.currentTimeMillis() - curr < 1200) {
            // do nothing
        }
        assertTrue(deb.checkPress(true));
    }

    @Test
    public void checkSetDelay() {
        long orig = deb.get_debounceDelay();
        deb.set_debounceDelay(5000);
        assertEquals(deb.get_debounceDelay(), 5000);
        deb.set_debounceDelay(orig);
    }
}
