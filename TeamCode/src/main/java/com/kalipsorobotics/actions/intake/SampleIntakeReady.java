package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleIntakeReady extends KActionSet{

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake) {
           this(intakeLinkageServoPos, intake, IntakeClaw.INTAKE_SMALL_SWEEP_INTAKE_READY_POS);
        }

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake, double intakeSmallSweepServoPos){
            KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
            moveBigPivot.setName("moveBigPivot");
            this.addAction(moveBigPivot);

            KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS);
            moveSmallPivot.setName("moveSmallPivot");
            this.addAction(moveSmallPivot);

            KServoAutoAction intakeArmBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
            intakeArmBigSweep.setName("intakeArmBigSweep");
            this.addAction(intakeArmBigSweep);

           KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), intakeSmallSweepServoPos);
           moveSmallSweep.setName("moveSmallSweep");
           moveSmallSweep.setDependentActions(moveBigPivot, moveSmallPivot);
           this.addAction(moveSmallSweep);

            KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), intakeLinkageServoPos);
            moveIntakeLSOut.setName("moveIntakeLSOut");
            moveIntakeLSOut.setDependentActions(moveBigPivot, moveSmallPivot, moveSmallSweep, intakeArmBigSweep);
            this.addAction(moveIntakeLSOut);
        }
    }
//intake ls extends
//check if big sweep is straight
//big pivot moves parallel to ground
//small pivot perpendicular to ground
//small sweep perpendicular to robot
//intake claw open

