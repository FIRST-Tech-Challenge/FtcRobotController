package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class UniConstants {

    //Lift
    public static final String SLIDE_MOTOR_LEFT = "LIFTL";
    public static final DcMotorEx.Direction SLIDE_MOTOR_LEFT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final String SLIDE_MOTOR_RIGHT = "LIFTR";
    public static final DcMotorEx.Direction SLIDE_MOTOR_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static int LIFT_TRANSFER = 500;
    public static int LIFT_MIDDLE = 500;
    public static int LIFT_BASKET = 1500;
    public static int LIFT_BAR = 1000;


    //Intake Claw
    //Servo for open and close
    public static final String INTAKE_CLAW_NAME = "ICLAW";
    public static final double INTAKE_CLAW_OPEN = 0.45;
    public static final double INTAKE_CLAW_CLOSED = 0.6;




    //Servo for horizontal rotation
    public static final String INTAKE_HORIZONTAL_NAME = "IHS";
    public static final double INTAKE_HORIZONTAL_PERP = 0.6;
    public static final double INTAKE_HORIZONTAL_PARA = 0.25;



    //Servo for vertical rotation
    public static final String INTAKE_VERTICAL_NAME = "IVS";
    public static double INTAKE_VERTICAL_DOWN = 0;
    public static double INTAKE_VERTICAL_UP = 0;



    //Outtake Claw
    public static final String OUTTAKE_CLAW_NAME = "OCLAW";
    public static final double OUTTAKE_CLAW_OPEN = 0;
    public static final double OUTTAKE_CLAW_CLOSED = .5;

    public static final String OUTTAKE_ROTATION_NAME = "ORS";
    public static final double OUTTAKE_ROTATION_PERP = 0;
    public static final double OUTTAKE_ROTATION_PARA = .5;


    //Drive
    public static final String DRIVE_FRONT_LEFT = "LFM";
    public static final String DRIVE_FRONT_RIGHT = "RFM";
    public static final String DRIVE_BACK_LEFT = "LRM";
    public static final String DRIVE_BACK_RIGHT = "RRM";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;


    //Outtake Pivot
    public static String OUTTAKE_PIVOT_LEFT_NAME = "";
    public static String OUTTAKE_PIVOT_RIGHT_NAME = "";

    public static final double OUTTAKE_PIVOT_LEFT_UP = 0;


    //Util Enums
    public enum loggingState {
        DISABLED,
        ENABLED,
        EXTREME
    }

    public enum EncoderReadingFrom{
        LEFT,
        RIGHT,
        BOTH
    }






}
