package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Characterization tests for {@link Debouncer}.
 *
 * <p>A Debouncer answers one question: "has this button been held down long enough that I
 * believe it?" Gamepad buttons and limit switches are noisy -- a single loop of "pressed"
 * might be electrical noise, not a human. The Debouncer waits until it has seen the input
 * true N times in a row before it reports true.
 *
 * <p>These tests describe what the code does TODAY. They are a safety net for the port to
 * BioBuzz: if the ported Debouncer still passes all of these, it still behaves the same.
 */
@DisplayName("Debouncer")
class DebouncerTest {

    /** Feed the same value in repeatedly, which is what a robot loop does. */
    private static void feed(Debouncer debouncer, boolean state, int times) {
        for (int i = 0; i < times; i++) {
            debouncer.In(state);
        }
    }

    @Nested
    @DisplayName("count threshold")
    class CountThreshold {

        @Test
        @DisplayName("stays false before the threshold is reached")
        void staysFalseBeforeThreshold() {
            Debouncer debouncer = new Debouncer(3);

            debouncer.In(true);
            assertFalse(debouncer.Out(), "1 of 3 true readings should not trigger");

            debouncer.In(true);
            assertFalse(debouncer.Out(), "2 of 3 true readings should not trigger");
        }

        @Test
        @DisplayName("becomes true exactly at the threshold")
        void becomesTrueAtThreshold() {
            Debouncer debouncer = new Debouncer(3);

            feed(debouncer, true, 3);

            assertTrue(debouncer.Out(), "3 of 3 true readings should trigger");
        }

        @Test
        @DisplayName("stays true while the input stays true (count does not overflow)")
        void staysTrueWhileHeld() {
            Debouncer debouncer = new Debouncer(3);

            feed(debouncer, true, 50);

            assertTrue(debouncer.Out(), "holding the button should keep reporting true");
        }

        @ParameterizedTest(name = "threshold of {0}")
        @ValueSource(ints = {1, 2, 5, 10})
        @DisplayName("honors whatever threshold it was constructed with")
        void honorsConstructedThreshold(int threshold) {
            Debouncer debouncer = new Debouncer(threshold);

            feed(debouncer, true, threshold - 1);
            assertFalse(debouncer.Out(), "should not trigger one reading early");

            debouncer.In(true);
            assertTrue(debouncer.Out(), "should trigger on the Nth reading");
        }

        @Test
        @DisplayName("a threshold of zero is already triggered before any input")
        void zeroThresholdStartsTriggered() {
            // Documents current behavior: Out() is (count >= 0), true from construction.
            // Not obviously intended, but nothing in Decode constructs a Debouncer(0).
            Debouncer debouncer = new Debouncer(0);

            assertTrue(debouncer.Out(), "threshold 0 means the count already satisfies it");
        }
    }

    @Nested
    @DisplayName("Used() suppression")
    class UsedSuppression {

        @Test
        @DisplayName("suppresses the output after the event is consumed")
        void suppressesAfterUse() {
            Debouncer debouncer = new Debouncer(2);
            feed(debouncer, true, 2);
            assertTrue(debouncer.Out());

            debouncer.Used();

            assertFalse(debouncer.Out(), "a consumed event should not fire again");
        }

        @Test
        @DisplayName("stays suppressed while the input remains true")
        void staysSuppressedWhileInputHeld() {
            Debouncer debouncer = new Debouncer(2);
            feed(debouncer, true, 2);
            debouncer.Used();

            feed(debouncer, true, 20);

            assertFalse(
                    debouncer.Out(),
                    "holding the button after consuming should not re-fire (no auto-repeat)");
        }

        @Test
        @DisplayName("can be called before the threshold and still suppresses")
        void suppressesEvenIfCalledEarly() {
            Debouncer debouncer = new Debouncer(3);
            debouncer.In(true);

            debouncer.Used();
            feed(debouncer, true, 2);

            assertFalse(debouncer.Out(), "Used() set early still gates the eventual trigger");
        }

        @Test
        @DisplayName("is idempotent")
        void isIdempotent() {
            Debouncer debouncer = new Debouncer(1);
            debouncer.In(true);
            debouncer.Used();
            debouncer.Used();

            assertFalse(debouncer.Out());
        }
    }

    @Nested
    @DisplayName("reset on a false input")
    class ResetOnFalse {

        @Test
        @DisplayName("a single false reading clears the accumulated count")
        void falseClearsCount() {
            Debouncer debouncer = new Debouncer(3);
            feed(debouncer, true, 2);

            debouncer.In(false);

            assertFalse(debouncer.Out());

            // The count restarted from zero, so two more true readings are still not enough.
            feed(debouncer, true, 2);
            assertFalse(debouncer.Out(), "count should have restarted from zero");

            debouncer.In(true);
            assertTrue(debouncer.Out(), "third reading after the reset triggers");
        }

        @Test
        @DisplayName("a false reading re-arms a consumed event")
        void falseClearsUsedFlag() {
            Debouncer debouncer = new Debouncer(2);
            feed(debouncer, true, 2);
            debouncer.Used();
            assertFalse(debouncer.Out());

            // Releasing the button is what re-arms it. This is the press/release/press cycle.
            debouncer.In(false);
            feed(debouncer, true, 2);

            assertTrue(debouncer.Out(), "releasing and pressing again should fire a new event");
        }

        @Test
        @DisplayName("stays false while the input stays false")
        void staysFalseWhileInputFalse() {
            Debouncer debouncer = new Debouncer(2);

            feed(debouncer, false, 10);

            assertFalse(debouncer.Out());
        }
    }

    @Nested
    @DisplayName("Force()")
    class Force {

        @Test
        @DisplayName("triggers the output with no input at all")
        void triggersImmediately() {
            Debouncer debouncer = new Debouncer(5);

            debouncer.Force();

            assertTrue(debouncer.Out(), "Force() should trigger without waiting for readings");
        }

        @Test
        @DisplayName("clears a previous Used() suppression")
        void clearsUsedFlag() {
            Debouncer debouncer = new Debouncer(2);
            feed(debouncer, true, 2);
            debouncer.Used();
            assertFalse(debouncer.Out());

            debouncer.Force();

            assertTrue(debouncer.Out(), "Force() should re-arm a consumed debouncer");
        }

        @Test
        @DisplayName("a forced event can still be consumed")
        void forcedEventCanBeUsed() {
            Debouncer debouncer = new Debouncer(3);
            debouncer.Force();

            debouncer.Used();

            assertFalse(debouncer.Out());
        }

        @Test
        @DisplayName("a false input cancels a forced trigger")
        void falseInputCancelsForce() {
            Debouncer debouncer = new Debouncer(3);
            debouncer.Force();

            debouncer.In(false);

            assertFalse(debouncer.Out(), "Force() is not sticky across a released input");
        }
    }
}
