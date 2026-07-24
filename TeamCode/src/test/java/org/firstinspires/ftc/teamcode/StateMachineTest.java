package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

/**
 * Requires (Gradle, TeamCode/build.gradle):
 * testImplementation 'junit:junit:4.13.2'
 * testImplementation 'pl.pragmatists:JUnitParams:1.1.1'
 * testImplementation 'org.mockito:mockito-core:5.7.0'
 *
 * No Android Gradle Plugin version bump needed — these run as ordinary
 * local JVM unit tests via ./gradlew :TeamCode:testDebugUnitTest.
 */
@RunWith(JUnitParamsRunner.class)
public class StateMachineTest {

    private enum TestState { A, B, C, D }

    @Before
    public void resetSingleton() {
        StateMachine.resetInstance();
    }

    private Command mockCommand() {
        Command cmd = mock(Command.class);
        when(cmd.isFinished()).thenReturn(false);
        return cmd;
    }

    // ---------- build() validation ----------

    @Test
    @Parameters({
            "false, false",
            "true, false"
    })
    public void build_throwsForInvalidConfiguration(boolean setInitial, boolean registerInitial) {
        StateMachine.Builder builder = StateMachine.builder()
                .state(TestState.A, mockCommand());

        if (setInitial && !registerInitial) {
            builder.initial(TestState.B); // never passed to .state()
        }

        assertThrows(IllegalStateException.class, builder::build);
    }

    @Test
    public void build_succeedsWithValidConfiguration() {
        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .initial(TestState.A)
                .build();

        assertNotNull(sm);
    }

    // ---------- initial state ----------

    @Test
    @Parameters({"A", "B", "C", "D"})
    public void initialize_setsCurrentStateToInitialAndInitializesItsCommand(String initialName) {
        TestState initial = TestState.valueOf(initialName);
        Command cmd = mockCommand();
        StateMachine sm = StateMachine.builder()
                .state(initial, cmd)
                .initial(initial)
                .build();

        sm.initialize();

        assertEquals(initial, sm.getCurrentState());
        verify(cmd, times(1)).initialize();
    }

    // ---------- isFinished() contract ----------

    @Test
    @Parameters({"true", "false"})
    public void isFinished_alwaysFalseRegardlessOfActiveCommand(boolean underlyingFinished) {
        Command cmd = mockCommand();
        when(cmd.isFinished()).thenReturn(underlyingFinished);

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, cmd)
                .initial(TestState.A)
                .build();

        sm.initialize();
        sm.execute();

        assertFalse(sm.isFinished());
    }

    // ---------- basic transition firing ----------

    @Test
    @Parameters({"true", "false"})
    public void execute_transitionsOnlyWhenConditionTrue(boolean conditionMet) {
        Command a = mockCommand();
        Command b = mockCommand();

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, a)
                .state(TestState.B, b)
                .initial(TestState.A)
                .transition(TestState.A, TestState.B, () -> conditionMet)
                .build();

        sm.initialize();
        sm.execute();

        if (conditionMet) {
            assertEquals(TestState.B, sm.getCurrentState());
            verify(a, times(1)).end(false);
            verify(b, times(1)).initialize();
        } else {
            assertEquals(TestState.A, sm.getCurrentState());
            verify(a, never()).end(anyBoolean());
            verify(b, never()).initialize();
        }
    }

    // ---------- transition ordering ----------

    @Test
    @Parameters(method = "transitionOrderCases")
    public void execute_firstMatchingLocalTransitionWinsOnTie(List<TestState> registrationOrder, TestState expected) {
        StateMachine.Builder builder = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .state(TestState.B, mockCommand())
                .state(TestState.C, mockCommand())
                .initial(TestState.A);

        for (TestState candidate : registrationOrder) {
            builder.transition(TestState.A, candidate, () -> true);
        }

        StateMachine sm = builder.build();
        sm.initialize();
        sm.execute();

        assertEquals(expected, sm.getCurrentState());
    }

    private Object[] transitionOrderCases() {
        return new Object[]{
                new Object[]{Arrays.asList(TestState.B, TestState.C), TestState.B},
                new Object[]{Arrays.asList(TestState.C, TestState.B), TestState.C}
        };
    }

    @Test
    @Parameters({"true", "false"})
    public void execute_globalTransitionTakesPriorityOverLocal(boolean globalConditionMet) {
        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .state(TestState.B, mockCommand())
                .state(TestState.C, mockCommand())
                .initial(TestState.A)
                .transition(TestState.A, TestState.B, () -> true)
                .anyTransition(TestState.C, () -> globalConditionMet)
                .build();

        sm.initialize();
        sm.execute();

        assertEquals(globalConditionMet ? TestState.C : TestState.B, sm.getCurrentState());
    }

    // ---------- enter/exit hooks ----------

    @Test
    @Parameters({"B", "C"})
    public void execute_enterAndExitHooksFireOnTransition(String targetName) {
        TestState target = TestState.valueOf(targetName);
        AtomicBoolean exitedA = new AtomicBoolean(false);
        AtomicBoolean enteredTarget = new AtomicBoolean(false);

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .state(TestState.B, mockCommand())
                .state(TestState.C, mockCommand())
                .initial(TestState.A)
                .transition(TestState.A, target, () -> true)
                .onExit(TestState.A, () -> exitedA.set(true))
                .onEnter(target, () -> enteredTarget.set(true))
                .build();

        sm.initialize();
        sm.execute();

        assertTrue(exitedA.get());
        assertTrue(enteredTarget.get());
    }

    @Test
    public void initialize_doesNotFireOnExitHookForInitialState() {
        AtomicBoolean exitFired = new AtomicBoolean(false);

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .initial(TestState.A)
                .onExit(TestState.A, () -> exitFired.set(true))
                .build();

        sm.initialize();

        assertFalse(exitFired.get());
    }

    // ---------- onStateChange listener ----------

    @Test
    @Parameters(method = "stateChangePairs")
    public void execute_stateChangeListenerReceivesFromAndToInOrder(TestState from, TestState to) {
        List<Object[]> events = new ArrayList<>();

        StateMachine sm = StateMachine.builder()
                .state(from, mockCommand())
                .state(to, mockCommand())
                .initial(from)
                .transition(from, to, () -> true)
                .onStateChange((f, t) -> events.add(new Object[]{f, t}))
                .build();

        sm.initialize();
        sm.execute();

        assertEquals(2, events.size());
        assertNull(events.get(0)[0]);
        assertEquals(from, events.get(0)[1]);
        assertEquals(from, events.get(1)[0]);
        assertEquals(to, events.get(1)[1]);
    }

    private Object[] stateChangePairs() {
        return new Object[]{
                new Object[]{TestState.A, TestState.B},
                new Object[]{TestState.C, TestState.D}
        };
    }

    // ---------- telemetry ----------

    @Test
    @Parameters({"A", "B", "C", "D"})
    public void execute_reportsCurrentStateToTelemetryWithoutCallingUpdate(String stateName) {
        TestState state = TestState.valueOf(stateName);
        Telemetry telemetry = mock(Telemetry.class);

        StateMachine sm = StateMachine.builder()
                .state(state, mockCommand())
                .initial(state)
                .telemetry(telemetry)
                .build();

        sm.initialize();
        sm.execute();

        verify(telemetry, atLeastOnce()).addData("StateMachine/currentState", state);
        verify(telemetry, never()).update();
    }

    @Test
    public void execute_neverTouchesTelemetryWhenNoneConfigured() {
        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .initial(TestState.A)
                .build();

        sm.initialize();
        sm.execute(); // must not throw with no telemetry configured
    }

    // ---------- error handling ----------

    @Test
    public void execute_throwsWhenTransitioningToUnregisteredState() {
        StateMachine sm = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .initial(TestState.A)
                .transition(TestState.A, TestState.B, () -> true) // B never registered
                .build();

        sm.initialize();

        assertThrows(IllegalStateException.class, sm::execute);
    }

    // ---------- singleton behavior ----------

    @Test
    public void getInstance_returnsSameInstanceUntilReset() {
        StateMachine first = StateMachine.getInstance();
        StateMachine second = StateMachine.getInstance();
        assertSame(first, second);

        StateMachine.resetInstance();
        StateMachine third = StateMachine.getInstance();
        assertNotSame(first, third);
    }

    @Test
    @Parameters({"1", "2", "3"})
    public void builder_alwaysConfiguresCurrentSingleton(int iteration) {
        StateMachine.resetInstance();
        StateMachine viaGetInstance = StateMachine.getInstance();
        StateMachine viaBuilder = StateMachine.builder()
                .state(TestState.A, mockCommand())
                .initial(TestState.A)
                .build();

        assertSame(viaGetInstance, viaBuilder);
    }

    // ---------- end() propagation ----------

    @Test
    @Parameters({"true", "false"})
    public void end_propagatesInterruptedFlagAndFiresExitHook(boolean interrupted) {
        Command cmd = mockCommand();
        AtomicBoolean exitFired = new AtomicBoolean(false);

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, cmd)
                .initial(TestState.A)
                .onExit(TestState.A, () -> exitFired.set(true))
                .build();

        sm.initialize();
        sm.end(interrupted);

        verify(cmd, times(1)).end(interrupted);
        assertTrue(exitFired.get());
    }

    // ---------- finished command without matching transition ----------

    @Test
    public void execute_finishedCommandDoesNotAutoAdvanceWithoutMatchingTransition() {
        Command cmd = mockCommand();
        when(cmd.isFinished()).thenReturn(true);

        StateMachine sm = StateMachine.builder()
                .state(TestState.A, cmd)
                .initial(TestState.A)
                .build();

        sm.initialize();
        sm.execute();

        assertEquals(TestState.A, sm.getCurrentState());
        verify(cmd, times(1)).end(false);
    }
}