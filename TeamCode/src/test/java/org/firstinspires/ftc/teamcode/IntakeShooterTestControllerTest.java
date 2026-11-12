package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

public class IntakeShooterTestControllerTest {
    @Test public void continuousFeedStartsImmediatelyAndIgnoresReadinessAndCycleTiming() {
        IntakeShooterTestController c = armed();
        c.configureShooting(0, true, .5, .35, .15);
        c.update(.1, false, false, false, false, false, true, true);
        c.update(1, false, false, true, true, false, false, false);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false)); // No gate-delay, home, speed, yaw or hood qualification.
        assertTrue(c.feedStage1());
        c.update(20, false, false, true, true, false, false, false);
        assertTrue(c.feedTransfer(false));
        assertEquals(1, c.firing().feedAttempts());
        c.update(21, false, false, true, false, false, false, false);
        assertEquals(FiringCoordinator.State.IDLE, c.firing().state());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(false));
        assertFalse(c.feedStage1());
        c.update(22, false, false, true, true, false, false, false);
        assertTrue(c.feedTransfer(false));
        c.update(23, false, false, false, true, false, false, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(false));
    }

    @Test public void continuousStopReverseAndSettingsChangesStillInhibitAllTransferOutputs() {
        IntakeShooterTestController c = armed();
        c.configureShooting(0, true, .4, .35, .15);
        c.update(.1, false, false, false, false, false, true, true);
        requestShot(c, 1, false);
        c.update(2, false, true, true, true, false, true, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedStage1());
        requestShot(c, 3, false);
        assertTrue(c.feedTransfer(false));
        c.update(4, false, false, true, true, true, true, false);
        requestShot(c, 5, false);
        assertFalse(c.feedTransfer(false));
        assertFalse(c.feedStage1());
        assertFalse(c.armed());
        c.update(6, false, false, false, false, false, true, false);
        requestShot(c, 7, false);
        assertTrue(c.feedTransfer(false));
        c.configureShooting(8, false, .4, .35, .15);
        assertFalse(c.feedTransfer(false));
        assertFalse(c.feedStage1());
    }

    @Test public void benchDefaultsAdvanceStage2AtPointOneStage1AtMidpointAndCloseEarlier() {
        IntakeShooterTestController c = armed();
        TransferController transfer = new TransferController();
        requestShot(c, 1, true);
        requestShot(c, GATE_OPEN_AT, true);
        requestShot(c, GATE_OPEN_AT + .099, true);
        assertFalse(c.feedTransfer(true));
        requestShot(c, GATE_OPEN_AT + .101, true);
        transfer.update(2, false, c.feedTransfer(true), true, c.feedStage1());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        requestShot(c, GATE_OPEN_AT + .249, true);
        assertFalse(c.feedStage1());
        requestShot(c, GATE_OPEN_AT + .251, true);
        transfer.update(2.1, true, c.feedTransfer(true), true, c.feedStage1());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        requestShot(c, GATE_OPEN_AT + .499, true);
        assertTrue(c.firing().gateOpen());
        requestShot(c, GATE_OPEN_AT + .502, true);
        transfer.update(3, true, c.feedTransfer(true), true, c.feedStage1());
        assertFalse(c.firing().gateOpen());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        // Other callers retain their shared competition/calibration defaults.
        assertEquals(.20, ShootingConfiguration.firing().gateTravelSeconds, 0);
        assertEquals(.50, ShootingConfiguration.firing().feedWindowSeconds, 0);
    }

    @Test public void modeAndTimingChangeStopsActiveFeedAndHeldTriggersCannotRestartIt() {
        IntakeShooterTestController c = feeding();
        assertTrue(c.configureShooting(2, true, .7, .8, .25));
        assertFalse(c.feedTransfer(true));
        assertFalse(c.firing().gateOpen());
        assertFalse(c.spinFlywheel());
        requestShot(c, 3, true);
        requestShot(c, 4, true);
        assertFalse(c.armed());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.configureShooting(4, true, .7, .8, .25));
        c.update(5, false, false, false, false, false, true, true);
        assertTrue(c.armed());
    }

    @Test public void customFeedRecoveryAndReadinessTimesAreObserved() {
        IntakeShooterTestController c = armed();
        c.configureShooting(0, false, .7, .8, .25);
        c.update(.1, false, false, false, false, false, true, true);
        requestShot(c, 1, true);
        requestShot(c, 1.24, true);
        assertFalse(c.firing().gateOpen());
        requestShot(c, 1.26, true);
        assertTrue(c.firing().gateOpen());
        requestShot(c, 1.37, true);
        requestShot(c, 1.659, true);
        assertFalse(c.feedStage1());
        requestShot(c, 1.661, true);
        assertTrue(c.feedStage1()); // Half of the custom 0.80 s open window.
        requestShot(c, 2.06, true);
        assertTrue(c.feedTransfer(true));
        requestShot(c, 2.08, true);
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        requestShot(c, 2.87, true);
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        requestShot(c, 2.89, true);
        assertEquals(FiringCoordinator.State.PREPARING, c.firing().state());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void zeroPausesAllowedAndInvalidTimingsRejectedWithoutChangingSettings() {
        IntakeShooterTestController c = armed();
        c.configureShooting(0, false, .05, 0, 0);
        for (double invalid : new double[] {-.1, Double.NaN, Double.POSITIVE_INFINITY}) {
            try {
                c.configureShooting(1, true, .5, invalid, 0);
                fail("Invalid pause accepted");
            } catch (IllegalArgumentException expected) { }
        }
        assertFalse(c.firing().continuous());
        assertEquals(.05, c.firing().feedWindowSeconds, 0);
        requestShot(c, 1, true);
        assertFalse(c.feedTransfer(true)); // Settings change still requires release.
        c.update(2, false, false, false, false, false, true, true);
        requestShot(c, 3, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true)); // Gate travel is never bypassed.
    }

    private static final double GATE_OPEN_AT = 1 + ShootingConfiguration.READY_STABLE_SECONDS + .01;
    private static final double FEED_AT = GATE_OPEN_AT + IntakeShooterTestController.GATE_TO_FEED_DELAY_SECONDS + .01;

    @Test public void stalledIntakeOscillatesForHalfSecondThenResumesAndCanRetry() {
        IntakeShooterTestController c = jammedIntake(false);
        assertEquals(-1, c.updateIntakePower(.25, false, 0), 0);
        assertEquals(1, c.updateIntakePower(.40, false, -100), 0);
        assertEquals(-1, c.updateIntakePower(.53, false, 100), 0);
        assertEquals(1, c.updateIntakePower(.65, false, -100), 0);
        assertTrue(c.isIntakeClearing());
        assertEquals(1, c.updateIntakePower(.75, false, 0), 0);
        assertFalse(c.isIntakeClearing());
        assertEquals(1, c.updateIntakePower(.94, false, 0), 0);
        assertEquals(-1, c.updateIntakePower(.96, false, 0), 0);
        assertTrue(c.isIntakeClearing());
    }

    @Test public void intakeRequiresContinuousLowSpeedAndAcceptsEitherEncoderSign() {
        IntakeShooterTestController c = armed();
        c.update(0, true, false, false, false, false, true, false);
        assertEquals(1, c.updateIntakePower(0, false, 0), 0);
        assertEquals(1, c.updateIntakePower(.19, false, 0), 0);
        assertFalse(c.isIntakeClearing());
        assertEquals(1, c.updateIntakePower(.20, false, -20), 0);
        assertEquals(1, c.updateIntakePower(.5, false, 20), 0);
        assertEquals(1, c.updateIntakePower(.6, false, 0), 0);
        assertEquals(1, c.updateIntakePower(.79, false, 0), 0);
        assertFalse(c.isIntakeClearing());
        assertEquals(-1, c.updateIntakePower(.81, false, 0), 0);
        assertTrue(c.isIntakeClearing());
    }

    @Test public void stopConflictingBumpersAndFeedCancelIntakeRecoveryImmediately() {
        for (int input = 1; input < 4; input++) {
            IntakeShooterTestController c = jammedIntake(false);
            c.update(.3, input == 1 || input == 2, input == 2,
                    input == 3, input == 3, input == 1, true, true);
            assertEquals(0, c.updateIntakePower(.3, false, 0), 0);
            assertFalse(c.isIntakeClearing());
            // After releasing all controls, a new load starts forward with a fresh stall window.
            c.update(.4, false, false, false, false, false, true, true);
            c.update(.5, true, false, false, false, false, true, true);
            assertEquals(1, c.updateIntakePower(.5, false, 0), 0);
            assertFalse(c.isIntakeClearing());
        }
    }

    @Test public void automaticLoadingRetainsJamRecoveryAfterForwardOverrideReleased() {
        IntakeShooterTestController c = jammedIntake(false);
        c.update(.3, false, false, false, false, false, true, false);
        assertTrue(c.loadIndex());
        assertEquals(-1, c.updateIntakePower(.3, false, 0), 0);
        assertTrue(c.isIntakeClearing());
        assertEquals(1, c.updateIntakePower(.4, false, 0), 0);
        assertEquals(1, c.updateIntakePower(.75, false, 100), 0);
        assertFalse(c.isIntakeClearing());
    }

    @Test public void recoveryHonorsReverseAndQueueFullAndCancelsOnDirectionChange() {
        IntakeShooterTestController c = jammedIntake(true);
        assertEquals(1, c.updateIntakePower(.25, false, 0), 0);
        assertEquals(-1, c.updateIntakePower(.40, false, 0), 0);
        assertEquals(1, c.updateIntakePower(.53, false, 0), 0);
        assertEquals(-1, c.updateIntakePower(.65, false, 0), 0);
        assertEquals(-1, c.updateIntakePower(.75, false, 0), 0);
        assertFalse(c.isIntakeClearing());

        c = jammedIntake(false);
        // RB keeps requesting forward even when full; releasing it stops automatic collection.
        assertEquals(1, c.intakePower(true), 0);
        c.update(.3, false, false, false, false, false, true, false);
        // A full queue stops the intake and cancels inward recovery immediately.
        assertEquals(0, c.updateIntakePower(.3, true, 0), 0);
        assertFalse(c.isIntakeClearing());
        assertEquals(0, c.updateIntakePower(.51, true, 0), 0); // A stopped intake cannot start recovery.
        assertFalse(c.isIntakeClearing());
        c.update(.55, false, true, false, false, false, true, false);
        assertEquals(-1, c.updateIntakePower(.55, true, 0), 0); // LB still reverses on request.
        c.update(.6, true, false, false, false, false, true, false);
        assertEquals(1, c.updateIntakePower(.6, false, 0), 0);
        assertFalse(c.isIntakeClearing());
    }

    @Test public void inhibitedAndUnarmedControlsCannotStartJamRecovery() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, true, false, false, false, false, true, false);
        assertEquals(0, c.updateIntakePower(0, false, 0), 0);
        assertEquals(0, c.updateIntakePower(5, false, 0), 0);
        assertFalse(c.isIntakeClearing());
        c = armed();
        // A feed request without an intake override pauses automatic loading.
        c.update(1, false, false, true, true, false, true, false);
        assertEquals(0, c.updateIntakePower(1, false, 0), 0);
        assertEquals(0, c.updateIntakePower(10, false, 0), 0);
        assertFalse(c.isIntakeClearing());
    }

    private IntakeShooterTestController jammedIntake(boolean reverse) {
        IntakeShooterTestController c = armed();
        c.update(0, !reverse, reverse, false, false, false, true, false);
        assertEquals(reverse ? -1 : 1, c.updateIntakePower(0, false, 0), 0);
        assertEquals(reverse ? 1 : -1, c.updateIntakePower(.25, false, 0), 0);
        assertTrue(c.isIntakeClearing());
        return c;
    }

    @Test public void heldTriggersRepeatTimedFeedsWithRecoveryAndReadinessBetweenShots() {
        IntakeShooterTestController c = feeding();
        TransferController transfer = new TransferController();
        double feedEnds = FEED_AT + IntakeShooterTestController.FEED_WINDOW_SECONDS;
        requestShot(c, feedEnds - .01, true);
        assertTrue(c.feedTransfer(true));
        double recoveryAt = feedEnds + .01;
        requestShot(c, recoveryAt, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        transfer.update(recoveryAt, true, c.feedTransfer(true), !c.loadIndex());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        requestShot(c, recoveryAt + ShootingConfiguration.RECOVERY_SECONDS - .01, true);
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        double recoveredAt = recoveryAt + ShootingConfiguration.RECOVERY_SECONDS + .01;
        requestShot(c, recoveredAt, true);
        requestShot(c, recoveredAt + .01, false);
        requestShot(c, recoveredAt + .2, false);
        assertEquals(1, c.firing().feedAttempts());
        assertFalse(c.firing().gateOpen());
        double readyAt = recoveredAt + .3;
        requestShot(c, readyAt, true);
        double reopenedAt = readyAt + ShootingConfiguration.READY_STABLE_SECONDS + .01;
        requestShot(c, reopenedAt, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, reopenedAt + IntakeShooterTestController.GATE_TO_FEED_DELAY_SECONDS + .01, true);
        assertTrue(c.feedTransfer(true));
        assertEquals(2, c.firing().feedAttempts());
    }

    @Test public void acceptedServoErrorCanFeedWithoutNeedingAnUnrequestedCorrection() {
        IntakeShooterTestController c = armed();
        // The servo controller accepts this 2-degree error without correcting it.
        boolean settled = IntakeShooterTestController.isYawSettled(2, 0);
        assertTrue(settled);
        requestShot(c, 1, settled);
        requestShot(c, GATE_OPEN_AT, settled);
        requestShot(c, FEED_AT, settled);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(true));
        assertFalse(c.feedTransfer(false));
    }

    @Test public void benchYawCheckRejectsExcessErrorMotionAndInvalidReadings() {
        assertTrue(IntakeShooterTestController.isYawSettled(4, -3));
        assertTrue(IntakeShooterTestController.isYawSettled(-4, 3));
        assertFalse(IntakeShooterTestController.isYawSettled(4.01, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(-4.01, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(0, 3.01));
        assertFalse(IntakeShooterTestController.isYawSettled(0, -3.01));
        assertFalse(IntakeShooterTestController.isYawSettled(Double.NaN, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(0, Double.POSITIVE_INFINITY));
        IntakeShooterTestController c = armed();
        requestShot(c, 1, IntakeShooterTestController.isYawSettled(5, 0));
        requestShot(c, 2, IntakeShooterTestController.isYawSettled(5, 0));
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    private IntakeShooterTestController armed() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, false, false, false, false, false, true, true);
        assertTrue(c.armed());
        return c;
    }

    private void requestShot(IntakeShooterTestController c, double now, boolean ready) {
        c.update(now, false, false, true, true, false, true, ready);
    }

    private IntakeShooterTestController feeding() {
        IntakeShooterTestController c = armed();
        requestShot(c, 1, true);
        requestShot(c, GATE_OPEN_AT, true);
        assertTrue(c.firing().gateOpen());
        requestShot(c, FEED_AT, true);
        assertTrue(c.feedTransfer(true));
        return c;
    }

    @Test public void heldControlsAtStartCannotMoveMotorsUntilReleased() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, true, false, true, true, false, true, true);
        c.update(1, true, false, true, true, false, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.loadIndex());
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        c.update(2, false, false, false, false, false, true, true);
        c.update(3, true, false, false, false, false, true, false);
        assertTrue(c.loadIndex());
        assertEquals(1, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
    }

    @Test public void loadingHonorsFullAndReverseOnlyRunsIntake() {
        IntakeShooterTestController c = armed();
        TransferController transfer = new TransferController();
        c.update(1, true, false, false, false, false, true, false);
        transfer.update(1, true, false, !c.loadIndex());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        assertEquals(1, c.intakePower(transfer.full()), 0);
        transfer.update(3.01, true, false, !c.loadIndex());
        assertTrue(transfer.full());
        assertEquals(1, c.intakePower(transfer.full()), 0); // RB overrides confirmed HIGH.
        assertEquals(1, transfer.stage1(), 0); // Stage-1 run-on seats the fourth ball.
        assertEquals(0, transfer.stage2(), 0);

        c.update(3.02, false, false, false, false, false, true, false);
        assertEquals(0, c.intakePower(transfer.full()), 0); // Release RB: automatic stop, not reverse.
        transfer.update(3.03, false, false, !c.loadIndex());
        assertFalse(transfer.full()); assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, c.intakePower(transfer.full()), 0); // LOW restores automatic forward.

        c.update(3.1, false, true, false, false, false, true, false);
        transfer.update(3.1, false, false, !c.loadIndex());
        assertFalse(c.loadIndex());
        assertEquals(-1, c.intakePower(true), 0);
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        c.update(3.2, true, true, false, false, false, true, false);
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.loadIndex());
    }

    @Test public void feedCannotBypassSpinHomeOrReadiness() {
        IntakeShooterTestController c = armed();
        c.update(1, true, false, false, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.loadIndex());
        assertEquals(1, c.intakePower(false), 0); // RB overrides intake direction during feed requests too.
        c.update(2, false, false, true, true, false, false, true);
        assertTrue(c.spinFlywheel());
        assertFalse(c.firing().gateOpen());
        requestShot(c, 3, false);
        requestShot(c, 4, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void readinessMustBeContinuousAndGateDelayMustElapseBeforeBothStagesRun() {
        IntakeShooterTestController c = armed();
        requestShot(c, 1, true);
        requestShot(c, 1.1, false);
        requestShot(c, 1.2, true);
        requestShot(c, 1.3, true);
        assertFalse(c.firing().gateOpen());
        requestShot(c, 1.36, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        double settledAt = 1.36 + IntakeShooterTestController.GATE_TO_FEED_DELAY_SECONDS;
        requestShot(c, settledAt - .01, true);
        assertFalse(c.feedTransfer(true));
        requestShot(c, settledAt + .01, true);
        assertFalse(c.feedTransfer(false));
        assertTrue(c.feedTransfer(true));
        TransferController transfer = new TransferController();
        transfer.update(settledAt + .01, true, c.feedTransfer(true), !c.loadIndex(), c.feedStage1());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        assertEquals(0, c.intakePower(false), 0);
    }

    @Test public void lossOfSpeedDuringTimedFeedClosesGateAndRequiresRecovery() {
        IntakeShooterTestController c = feeding();
        requestShot(c, FEED_AT + .01, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        requestShot(c, 3, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.spinFlywheel());
        assertEquals(FiringCoordinator.State.PREPARING, c.firing().state());
    }

    @Test public void releasingThenReholdingFeedRequiresInitialReadinessAgain() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, false, false, true, true);
        requestShot(c, 2, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, 3, true);
        requestShot(c, 3.16, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, 3.16 + IntakeShooterTestController.GATE_TO_FEED_DELAY_SECONDS + .01, true);
        assertTrue(c.feedTransfer(true));
    }

    @Test public void lostHomeStillStopsTimedFeed() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, true, false, false, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void releasingEitherTriggerImmediatelyClosesGateAndStopsFeeding() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, false, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.spinFlywheel());
        c = feeding();
        c.update(FEED_AT + .01, false, false, false, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.spinFlywheel());
    }

    @Test public void stopWinsOverEveryHoldAndRequiresReleaseBeforeResuming() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, true, true, true, true, true, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.loadIndex());
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
        c.update(2, true, false, true, true, false, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.feedTransfer(true));
        c.update(3, false, false, false, false, false, true, true);
        assertTrue(c.armed());
    }

    @Test public void reverseCancelsAnActiveFeedEvenWhenTriggersStayHeld() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, true, true, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.loadIndex());
        assertEquals(-1, c.intakePower(false), 0);
    }
}
