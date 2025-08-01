package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Names {

    //Lift
    public static final String SLIDE_MOTOR_LEFT = "SML";
    public static final DcMotorEx.Direction SLIDE_MOTOR_LEFT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final String SLIDE_MOTOR_RIGHT = "SMR";
    public static final DcMotorEx.Direction SLIDE_MOTOR_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;

    //Intake Claw
    public static final String INTAKE_CLAW = "Claw";
    public static final double INTAKE_CLAW_OPEN = 0;
    public static final double INTAKE_CLAW_CLOSED = .5;

    public static final String INTAKE_DOWN = "Down";
    public static final double INTAKE_DOWN_DOWN = 0;
    public static final double INTAKE_DOWN_UP = .5;

    public static final String INTAKE_SIDE = "Side";
    public static final double INTAKE_SIDE_PERP = 0;
    public static final double INTAKE_SIDE_PARA = .5;


    //Outtake Claw
    public static final String OUTTAKE_CLAW = "OuttakeClaw";
    public static final double OUTTAKE_CLAW_OPEN = 0;
    public static final double OUTTAKE_CLAW_CLOSED = .5;

    public static final String CLAW_ROTATION = "CR1";
    public static final double CLAW_ROTATION_PERP = 0;
    public static final double CLAW_ROTATION_PARA = .5;


    public static final String DRIVE_FRONT_LEFT = "FL";
    public static final String DRIVE_FRONT_RIGHT = "FR";
    public static final String DRIVE_BACK_LEFT = "BL";
    public static final String DRIVE_BACK_RIGHT = "BR";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;







}
