package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IntakeClaw;

public class SpecimenSweepingRoundTrip extends KActionSet {

    public SpecimenSweepingRoundTrip(DriveTrain driveTrain, IntakeClaw intake, WheelOdometry wheelOdometry, double sampleY) {

        //pure pursuit to sample 1 to depot (-550, -200, heading [-120]) + intake ready
        //intake sample 1
        //intake ready (claw closed)
        //pure pursuit heading [-45]
        //open claw

        PurePursuitAction moveToSample = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample.setName("moveFloorSamples");
        this.addAction(moveToSample);
        moveToSample.addPoint(-615,sampleY,-120);

        SampleIntakeReady intakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intake, IntakeClaw.INTAKE_SMALL_SWEEP_SPECIMEN_POS);
        intakeReady.setName("intakeReady");
        this.addAction(intakeReady);

        SampleIntakeAction intakeAction = new SampleIntakeAction(intake);
        intakeAction.setName("intakeAction");
        intakeAction.setDependentActions(intakeReady, moveToSample);
        this.addAction(intakeAction);

        SampleIntakeReady intakeReadyClosed = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intake, IntakeClaw.INTAKE_SMALL_SWEEP_SPECIMEN_POS);
        intakeReadyClosed.setName("intakeReadyClosed");
        intakeReadyClosed.setDependentActions(intakeAction);
        this.addAction(intakeReadyClosed);

        PurePursuitAction sweepToDepot = new PurePursuitAction(driveTrain, wheelOdometry);
        sweepToDepot.setName("moveFloorSamples");
        sweepToDepot.setDependentActions(intakeReadyClosed);
        this.addAction(sweepToDepot);
        sweepToDepot.addPoint(-550,sampleY,-45);

        KServoAutoAction openClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        openClaw.setDependentActions(sweepToDepot);
        this.addAction(openClaw);


    }
}
