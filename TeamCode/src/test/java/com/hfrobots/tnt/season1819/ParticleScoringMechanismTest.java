package com.hfrobots.tnt.season1819;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.hfrobots.tnt.fakes.FakeTelemetry;

import junit.framework.Assert;

import org.junit.Before;
import org.junit.Test;

public class ParticleScoringMechanismTest {
    private ParticleScoringMechanism particleScoringMechanism;

    private FakeServo boxTipServo = new FakeServo();

    private FakeDcMotorEx elevatorMotor = new FakeDcMotorEx();

    private FakeOnOffButton elevatorCommandDownButton = new FakeOnOffButton();

    private FakeOnOffButton elevatorCommandUpButton = new FakeOnOffButton();

    private FakeOnOffButton emergencyStopButton = new FakeOnOffButton();

    private FakeOnOffButton elevatorLowerLimitButton = new FakeOnOffButton();

    private FakeOnOffButton elevatorUpperLimitButton = new FakeOnOffButton();

    private FakeOnOffButton elevatorBoxTipButton = new FakeOnOffButton();

    private FakeOnOffButton limitOverrideButton = new FakeOnOffButton();

    @Before
    public void setUp() {
        particleScoringMechanism = ParticleScoringMechanism.builder().boxTipServo(boxTipServo)
                .elevatorMotor(elevatorMotor)
                .boxTipButton(elevatorBoxTipButton)
                .elevatorCommandDownButton(elevatorCommandDownButton)
                .elevatorCommandUpButton(elevatorCommandUpButton)
                .elevatorEmergencyStopButton(new DebouncedButton(emergencyStopButton))
                .elevatorLowerLimitButton(new DebouncedButton(elevatorLowerLimitButton))
                .elevatorUpperLimitButton(new DebouncedButton(elevatorUpperLimitButton))
                .limitOverrideButton(limitOverrideButton)
                .telemetry(new FakeTelemetry()).build();

    }

    @Test
    public void closedLoopControl() {
        /* Test all of the states, and their transitions, that can happen under closed loop control.
         * we start at the lower limit and "run" until we reach slightly above the upper limit. we
         * then test that the code thinks that we are at the upper limit. We then test if tipping the
         * box will work at the upper limit. we then "run" the elavtor back to the lower limit and
         * test that we have reached the lower limit.
         */

        // Starting state
        closedLoopStartingState();

        // Go from lower limit to upper limit
        closedLoopLowerLimitToUpperLimit();

        // Upper limit to lower limit
        closedLoopUpperLimitToLowerLimit();
    }

    private void closedLoopLowerLimitToUpperLimit() {

        // Cause a transition to another state
        pressButtonThenDoPeriodicTask(elevatorUpperLimitButton);

        particleScoringMechanism.doPeriodicTask(); // causes new state to run once

        //  Assert what state the scoring mechanism is in
        Assert.assertTrue(particleScoringMechanism.currentStateIsClosedLoopUp());

        //  Assert what motor power we expect
        Assert.assertTrue(elevatorMotor.getPower() > 0);

        //  Assert what box position we expect
        Assert.assertTrue(particleScoringMechanism.isBoxtipServoInHoldingPosition());

        // test that we are at upper limit

        elevatorMotor.setCurrentPosistion(1551);
        particleScoringMechanism.doPeriodicTask(); // causes state to change

        // tests if scoring mech is at the upper limit (1550)
        Assert.assertTrue(particleScoringMechanism.isAtUpperLimit());

        //  Assert what motor power we expect
        Assert.assertEquals(elevatorMotor.getPower(), 0 , 0.01);

        //  Assert what box position we expect
        Assert.assertTrue(particleScoringMechanism.isBoxtipServoInHoldingPosition());

        // tip scoring box

        // Changes states of Box Tip Button
        pressButtonThenDoPeriodicTask(elevatorBoxTipButton);

        Assert.assertTrue(particleScoringMechanism.isBoxtipServoInScoringPosition());
    }

    private void closedLoopStartingState() {
        //  Assert what state the scoring mechanism is in
        Assert.assertTrue(particleScoringMechanism.currentStateIsAtLowerLimit());

        //  Assert what motor power we expect
        Assert.assertEquals(elevatorMotor.getPower(), 0 , 0.01);

        //  Assert what box position we expect
        Assert.assertTrue(particleScoringMechanism.isBoxtipServoInLoadingPosition());
    }

    private void closedLoopUpperLimitToLowerLimit() {
        // Cause a transition to another state
        pressButtonThenDoPeriodicTask(elevatorLowerLimitButton);

        particleScoringMechanism.doPeriodicTask(); // causes new state to run once

        //  Assert what state the scoring mechanism is in
        Assert.assertTrue(particleScoringMechanism.currentStateIsClosedLoopDown());

        //  Assert what motor power we expect
        Assert.assertTrue(elevatorMotor.getPower() < 0);

        //  Assert what box position we expect
        Assert.assertTrue(particleScoringMechanism.isBoxtipServoInHoldingPosition());

        // Are we at the lower limit

        // go to lower limit
        elevatorMotor.setCurrentPosistion(0);
        particleScoringMechanism.doPeriodicTask(); // causes state to change

        // test that we are at the lower limit
        Assert.assertTrue(particleScoringMechanism.isAtLowerLimit());
    }

    private void pressButtonThenDoPeriodicTask(FakeOnOffButton button) {
        button.setPressed(true);
        particleScoringMechanism.doPeriodicTask(); // causes state to change
        button.setPressed(false);
    }
}
