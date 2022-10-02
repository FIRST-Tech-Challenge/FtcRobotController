package org.firstinspires.ftc.teamcode.Components;

public class StateMachine{
    public StateMachine(){

    }
    public enum ClawStates {
        CLAW_CLOSED(true, "CLAW_CLOSED"),
        CLAW_CLOSING(false, "CLAW_CLOSING"),
        CLAW_OPEN(false, "CLAW_OPEN"),
        CLAW_OPENING(false, "CLAW_OPENING");

        boolean status;
        String name;

        ClawStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }
    public enum AlignerStates {
        ALIGNER_SLIDES_EXTENDING(false, "ALIGNER_SLIDES_EXTENDING"),
        ALIGNER_SLIDES_RETRACTING(false, "ALIGNER_SLIDES_RETRACTING"),
        ALIGNER_SLIDES_EXTENDED(false, "ALIGNER_SLIDES_EXTENDED"),
        ALIGNER_SLIDES_RETRACTED(false, "ALIGNER_SLIDES_RETRACTED"),
        ALIGNER_SLIDES_STILL(true, "ALIGNER_SLIDES_STILL"),
        ALIGNER_INTAKE_STILL(true, "ALIGNER_INTAKE_STILL"),
        ALIGNER_INTAKE_INTAKING(false, "ALIGNER_INTAKE_INTAKING"),
        ALIGNER_INTAKE_REVERSING(false, "ALIGNER_INTAKE_REVERSING");

        boolean status;
        String name;

        AlignerStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }
    public enum LiftStates {
        LIFT_GROUND(true, "LIFT_GROUND"),
        LIFT_GROUND_RAISING(false, "LIFT_GROUND_RAISING"),
        LIFT_GROUND_JUNCTION(false, "LIFT_GROUND_JUNCTION"),
        LIFT_GROUND_JUNCTION_RAISING(false, "LIFT_GROUND_JUNCTION_RAISING"),
        LIFT_GROUND_JUNCTION_LOWERING(false, "LIFT_GROUND_JUNCTION_LOWERING"),
        LIFT_LOW(false, "LIFT_LOW"),
        LIFT_LOW_RAISING(false, "LIFT_LOW_RAISING"),
        LIFT_LOW_LOWERING(false, "LIFT_LOW_LOWERING"),
        LIFT_MID(false, "LIFT_MID"),
        LIFT_MID_RAISING(false, "LIFT_MID_RAISING"),
        LIFT_MID_LOWERING(false, "LIFT_MID_LOWERING"),
        LIFT_HIGH(false, "LIFT_HIGH"),
        LIFT_HIGH_LOWERING(false, "LIFT_HIGH_LOWERING");

        boolean status;
        String name;

        LiftStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }
    public enum ClawExtensionStates {
        CLAW_EXTENDED(false, "CLAW_EXTENDED"),
        CLAW_EXTENDING(false, "CLAW_EXTENDING"),
        CLAW_RETRACTING(true,"CLAW_RETRACTING"),
        CLAW_RETRACTED(true,"CLAW_RETRACTED");

        boolean status;
        String name;

        ClawExtensionStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }
}
