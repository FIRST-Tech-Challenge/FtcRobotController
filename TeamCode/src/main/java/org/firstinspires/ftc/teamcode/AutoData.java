package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// TODO: Find better file location for this

@Config
public class AutoData {
    // Timing variables
    public static double CLAW_CLOSE_OFFSET = 0.05;
    public static double INTAKE_LIFT_OFFSET = 0.565;
    public static double INTAKE_DELAY = .58;

    public static double LOWER_OFFSET = 0.08;
    public static double DEPOSIT_DELAY = 0.4;
    public static double DEPOSIT_OFFSET = 0.2;
    public static double RETRACT_OFFSET = 0.1;

    // Positions in centimeters/degrees of where to intake/deposit
    public static double INTAKE_X = 158.25;
    public static double INTAKE_Y = -26.98;

    // Depositing positon
    public static double DEPOSIT_X = 87.97;
    public static double DEPOSIT_Y = -17.29;
    public static double DEPOSIT_ANGLE = 139.94;
    public static double DEPOSIT_ANGLE_ADJUSTMENT = .0012;

    public static int DEPOSIT_DROP_AMOUNT = 800;

    // DEPRICATED
    public static int INTAKING_START_POS = 420;
    public static int INTAKING_DECREMENT = 55;

    public static int AUTO_INTAKE_LIFT_HEIGHT_1 = 373;
    public static int AUTO_INTAKE_LIFT_HEIGHT_2 = 297;
    public static int AUTO_INTAKE_LIFT_HEIGHT_3 = 182;
    public static int AUTO_INTAKE_LIFT_HEIGHT_4 = 112;
    public static int AUTO_INTAKE_LIFT_HEIGHT_5 = 0;
}
