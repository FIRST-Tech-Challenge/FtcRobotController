package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class SampleIntakeReady extends KActionSet{

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake) {
           this(intakeLinkageServoPos, intake, IntakeClaw.INTAKE_SMALL_SWEEP_INTAKE_READY_POS);
        }

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake, double intakeSmallSweepServoPos) {
            this(intakeLinkageServoPos, intake, intakeSmallSweepServoPos, IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN);
        }

        public SampleIntakeReady(double intakeLinkageServoPos, IntakeClaw intake, double intakeSmallSweepServoPos,
                                 double intakeClawPos) {

//            WaitAction smallPivotHeadStart = new WaitAction(200);
//            smallPivotHeadStart.setName("smallPivotHeadStart");
//            this.addAction(smallPivotHeadStart);

            KServoAutoAction pushRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
                    IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
            pushRatchet.setName("pushRatchet");
            this.addAction(pushRatchet);

            KServoAutoAction moveOpenClaw = new KServoAutoAction(intake.getIntakeClawServo(), intakeClawPos);
            moveOpenClaw.setName("moveOpenClaw");
            this.addAction(moveOpenClaw);

            KServoAutoAction moveBigPivot1 = new KServoAutoAction(intake.getIntakeBigPivotServo(),
                    IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS);
            moveBigPivot1.setName("moveBigPivot1");
            this.addAction(moveBigPivot1);
            //moveBigPivot1.setDependentActions(smallPivotHeadStart);

            KServoAutoAction moveSmallPivot1 = new KServoAutoAction(intake.getIntakeSmallPivotServo(),
                    IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS);
            moveSmallPivot1.setName("moveSmallPivot1");
            this.addAction(moveSmallPivot1);

            KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
            moveBigPivot2.setName("moveBigPivot2");
            this.addAction(moveBigPivot2);
            moveBigPivot2.setDependentActions(moveSmallPivot1, moveBigPivot1);

            KServoAutoAction moveSmallPivot2 = new KServoAutoAction(intake.getIntakeSmallPivotServo(),
                    IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS);
            moveSmallPivot2.setName("moveSmallPivot2");
            this.addAction(moveSmallPivot2);
            moveSmallPivot2.setDependentActions(moveBigPivot2);

            KServoAutoAction intakeArmBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
            intakeArmBigSweep.setName("intakeArmBigSweep");
            this.addAction(intakeArmBigSweep);

            KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), intakeSmallSweepServoPos);
            moveSmallSweep.setName("moveSmallSweep");
//            moveSmallSweep.setDependentActions(moveBigPivot, moveSmallPivot);
            this.addAction(moveSmallSweep);

            KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), intakeLinkageServoPos);
            moveIntakeLSOut.setName("moveIntakeLSOut");
            this.addAction(moveIntakeLSOut);
            //moveIntakeLSOut.setDependentActions(moveSmallPivot1);

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

