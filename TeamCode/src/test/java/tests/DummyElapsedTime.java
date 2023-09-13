package tests;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class DummyElapsedTime extends ElapsedTime {
    double fakeSeconds;

    public DummyElapsedTime(double fakeSeconds) {
        this.fakeSeconds = fakeSeconds;
    }

    public DummyElapsedTime() {
        this.fakeSeconds = 0;
    }

    public void setFakeSeconds(double seconds) {
        this.fakeSeconds = seconds;
    }

    @Override
    public double seconds() {
        return this.fakeSeconds;
    }

    @Test
    static void DummyElapsedTimeFakeSecondsCheck() {
        DummyElapsedTime det = new DummyElapsedTime(0);
        det.setFakeSeconds(20);
        assertEquals(20,det.seconds());
    }
}
