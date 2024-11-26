package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;

public class Macros {
    public static HobbesState EXTENDO_BEFORE_PICKUP = new HobbesState(EXTENDO_OUT_SOME, null, null, null, null, INTAKE_POWER, null, null, null);

    public static HobbesState EXTENDO_ARM_WRIST_FLAT = new HobbesState(null, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null);

    public static HobbesState EXTENDO_ARM_WRIST_UP = new HobbesState(null, EXTENDO_ARM_UP, EXTENDO_WRIST_UP, null, null, INTAKE_OFF, null, null, null);

    public static HobbesState EXTENDO_ARM_WRIST_ANGLED =  new HobbesState(null, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null);

    public static HobbesState FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, null);

    public static HobbesState TRANSFER_CLOSED = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN, null);
    public static HobbesState TRANSFER_ON =  new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_CLOSED, 250));
    public static HobbesState TRANSFER_WRIST_UP =  new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_ON, 200));
    public static HobbesState FULL_TRANSFER = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_WRIST_UP, 600));

    public static HobbesState SLIDES_DOWN = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN, null);

    public static HobbesState SLIDES_DEPOSIT2 = new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null, null);
    public static HobbesState SLIDES_DEPOSIT =  new HobbesState(null, null, null, null, null, null, null, SLIDES_OUT_TOP_SAMPLE, new LinkedState(SLIDES_DEPOSIT2, 1200));

    public static HobbesState OPEN_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null, null);

    public static HobbesState CLOSE_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, null);

    public static HobbesState EXTEND = new HobbesState(EXTENDO_OUT_SOME+0.1, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null);

}
