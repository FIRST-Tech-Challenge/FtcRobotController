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
}
