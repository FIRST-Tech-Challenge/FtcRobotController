package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleIntakeReady extends KActionSet{

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake) {
           this(intakeLinkageServoPos, intake, IntakeClaw.INTAKE_SMALL_SWEEP_INTAKE_READY_POS);
        }

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake, double intakeSmallSweepServoPos) {
            this(intakeLinkageServoPos, intake, intakeSmallSweepServoPos, IntakeClaw.INTAKE_CLAW_OPEN);
        }

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake, double intakeSmallSweepServoPos,
                                 double intakeClawPos){

            KServoAutoAction moveOpenClaw = new KServoAutoAction(intake.getIntakeClawServo(), intakeClawPos);
            moveOpenClaw.setName("moveOpenClaw");
            this.addAction(moveOpenClaw);

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

            KServoAutoAction pushRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
                    IntakeClaw.INTAKE_RATCHET_PUSH_POS);
            pushRatchet.setName("pushRatchet");
            this.addAction(pushRatchet);

            KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), intakeLinkageServoPos);
            moveIntakeLSOut.setName("moveIntakeLSOut");
            this.addAction(moveIntakeLSOut);

//            KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), intakeLinkageServoPos);
//            moveIntakeLSOut.setName("moveIntakeLSOut");
//            moveIntakeLSOut.setDependentActions(moveBigPivot, moveSmallPivot, moveSmallSweep, intakeArmBigSweep, moveOpenClaw);
//            this.addAction(moveIntakeLSOut);

        }
    }
//intake ls extends
//check if big sweep is straight
//big pivot moves parallel to ground
//small pivot perpendicular to ground
//small sweep perpendicular to robot
//intake claw open

