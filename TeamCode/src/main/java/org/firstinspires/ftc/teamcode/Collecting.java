package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.HMapConfig;

public class Collecting {

    Telemetry   logger;

    boolean     isIntakeReady;
    boolean     isOuttakeReady;
    boolean     areOuttakeSlidesReady;
    boolean     areIntakeSlidesReady;

    DcMotor     intakeSlidesMotor;
    DcMotor     outtakeSlidesLeftMotor;
    DcMotor     outtakeSlidesRightMotor;

    Servo       intakeArmLeftPitchServo;
    Servo       intakeArmRightPitchServo;
    Servo       intakeElbowPitchServo;
    Servo       intakeWristRollServo;
    Servo       intakeClawServo;

    Servo       outtakeElbowRightPitchServo;
    Servo       outtakeElbowLeftPitchServo;
    Servo       outtakeWristRollServo;
    Servo       outtakeClawServo;

    Gamepad gamepad;

    public void setHW(HMapConfig config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        isIntakeReady = true;
        isOuttakeReady = true;
        areOuttakeSlidesReady = true;
        areIntakeSlidesReady = true;

        logger = tm;

        String intakeSlides  = config.INTAKE_SLIDES();
        String outtakeLeftSlides = config.OUTTAKE_SLIDES_LEFT();
        String outtakeRightSlides = config.OUTTAKE_SLIDES_RIGHT();

        String intakeArmLeftPitch = config.INTAKE_ARM_PITCH_LEFT();
        String intakeArmRightPitch = config.INTAKE_ARM_PITCH_RIGHT();
        String intakeElbowPitch = config.INTAKE_ELBOW_PITCH();
        String intakeWristRoll = config.INTAKE_WRIST_ROLL();
        String intakeClaw = config.INTAKE_CLAW();
        String outtakeElbowRightPitch = config.OUTTAKE_ELBOW_PITCH_RIGHT();
        String outtakeElbowLeftPitch = config.OUTTAKE_ELBOW_PITCH_LEFT();
        String outtakeWristRoll = config.OUTTAKE_WRIST_ROLL();
        String outtakeClaw = config.OUTTAKE_CLAW();

        logger.addLine("=== COLLECTING ===");

        String status = "--> CONF IN : ";
        if(intakeSlides.length()           == 0) { status += " SL: KO" ;            isIntakeReady = false;  }
        else                                     { status += " SL: OK"; }
        if(intakeArmLeftPitch.length()     == 0) { status += " APL: KO" ;           isIntakeReady = false;  }
        else                                     { status += " APL: OK"; }
        if(intakeArmRightPitch.length()    == 0) { status += " APR: KO" ;           isIntakeReady = false;  }
        else                                     { status += " APR: OK"; }
        if(intakeElbowPitch.length()       == 0) { status += " EP: KO" ;            isIntakeReady = false;  }
        else                                     { status += " EP: OK"; }
        if(intakeWristRoll.length()        == 0) { status += " WR: KO" ;            isIntakeReady = false;  }
        else                                     { status += " WR: OK"; }
        if(intakeClaw.length()             == 0) { status += " CW: KO" ;            isIntakeReady = false;  }
        else                                     { status += " CW: OK"; }
        logger.addLine(status);

        status = "--> CONF OUT : ";
        if(outtakeLeftSlides.length()      == 0) { status += " SLL: KO" ;           isOuttakeReady = false;  }
        else                                     { status += " SLL: OK"; }
        if(outtakeRightSlides.length()     == 0) { status += " SLR: KO" ;           isOuttakeReady = false;  }
        else                                     { status += " SLR: OK"; }
        if(outtakeElbowLeftPitch.length()  == 0) { status += " EPL: KO" ;           isOuttakeReady = false;  }
        else                                     { status += " EPL: OK"; }
        if(outtakeElbowRightPitch.length() == 0) { status += " EPR: KO" ;           isOuttakeReady = false;  }
        else                                     { status += " EPR: OK"; }
        if(outtakeWristRoll.length()       == 0) { status += " WR: KO" ;            isOuttakeReady = false;  }
        else                                     { status += " WR: OK"; }
        if(outtakeClaw.length()            == 0) { status += " CW: KO" ;            isIntakeReady = false;  }
        else                                     { status += " CW: OK"; }
        logger.addLine(status);

        if (isIntakeReady) {
            intakeSlidesMotor = hwm.tryGet(DcMotor.class, intakeSlides);
            intakeArmLeftPitchServo = hwm.tryGet(Servo.class, intakeArmLeftPitch);
            intakeArmRightPitchServo = hwm.tryGet(Servo.class, intakeArmRightPitch);
            intakeElbowPitchServo = hwm.tryGet(Servo.class, intakeElbowPitch);
            intakeWristRollServo = hwm.tryGet(Servo.class, intakeWristRoll);
            intakeClawServo = hwm.tryGet(Servo.class, intakeClaw);

            status = "--> HW IN : ";
            if(intakeSlidesMotor          == null)  { status += " SL: KO" ;   }
            else                                    { status += " SL: OK"; }
            if(intakeArmLeftPitchServo    == null)  { status += " APL: KO" ;   }
            else                                    { status += " APL: OK"; }
            if(intakeArmRightPitchServo   == null)  { status += " APR: KO" ; }
            else                                    { status += " APR: OK"; }
            if(intakeElbowPitchServo      == null)  { status += " EP: KO" ; }
            else                                    { status += " EP: OK"; }
            if(intakeWristRollServo       == null)  { status += " WR: KO" ;  }
            else                                    { status += " WR: OK"; }
            if(intakeClawServo            == null)  { status += " CW: KO" ;  }
            else                                    { status += " CW: OK"; }
            logger.addLine(status);

        }

        if(isOuttakeReady) {
            outtakeSlidesLeftMotor = hwm.tryGet(DcMotor.class,outtakeLeftSlides);
            outtakeSlidesRightMotor = hwm.tryGet(DcMotor.class,outtakeRightSlides);
            outtakeElbowRightPitchServo = hwm.tryGet(Servo.class,outtakeElbowRightPitch);
            outtakeElbowLeftPitchServo = hwm.tryGet(Servo.class,outtakeElbowLeftPitch);
            outtakeWristRollServo = hwm.tryGet(Servo.class,outtakeWristRoll);
            outtakeClawServo = hwm.tryGet(Servo.class,outtakeClaw);

            status = "--> HW OUT : ";
            if(outtakeSlidesLeftMotor == null)       { status += " SLL: KO" ;}
            else                                     { status += " SLL: OK"; }
            if(outtakeRightSlides.length()     == 0) { status += " SLR: KO" ;}
            else                                     { status += " SLR: OK"; }
            if(outtakeElbowLeftPitch.length()  == 0) { status += " EPL: KO"; }
            else                                     { status += " EPL: OK"; }
            if(outtakeElbowRightPitch.length() == 0) { status += " EPR: KO"; }
            else                                     { status += " EPR: OK"; }
            if(outtakeWristRoll.length()       == 0) { status += " WR: KO" ; }
            else                                     { status += " WR: OK";  }
            if(outtakeClaw.length()            == 0) { status += " CW: KO" ; }
            else                                     { status += " CW: OK";  }
            logger.addLine(status);

        }

        if (isIntakeReady) {
            if(intakeSlidesMotor == null) { areIntakeSlidesReady = false; }

            if (config.INTAKE_SLIDES_REVERSE() && intakeSlidesMotor != null) {
                intakeSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

        }
        if(isOuttakeReady) {
            if(outtakeSlidesLeftMotor == null) { areOuttakeSlidesReady = false; }
            if(outtakeSlidesRightMotor == null) { areOuttakeSlidesReady = false; }

            if (config.OUTTAKE_SLIDES_LEFT_REVERSE() && outtakeSlidesLeftMotor != null) {
                outtakeSlidesLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (config.OUTTAKE_SLIDES_RIGHT_REVERSE() && outtakeSlidesRightMotor != null) {
                outtakeSlidesRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (outtakeSlidesLeftMotor != null) { outtakeSlidesLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
            if (outtakeSlidesRightMotor != null) { outtakeSlidesRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
            if (outtakeSlidesLeftMotor != null) { outtakeSlidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
            if (outtakeSlidesRightMotor != null) { outtakeSlidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
        }

        gamepad = gp;

        status = "--> READY INTAKE :";
        if(isIntakeReady) {
            if(areIntakeSlidesReady) { status += " SL: OK"; }
            else { status += " SL: KO"; }
        }
        else { status += " KO"; }
        logger.addLine(status);


        status = "--> READY OUTTAKE :";
        if(isOuttakeReady) {
            if(areOuttakeSlidesReady) { status += " SL : OK"; }
            else { status += " SL : KO"; }
        }
        else { status += " KO"; }
        logger.addLine(status);
    }

    public void move() {

        // Outtake slides control
        if (gamepad.left_bumper && areOuttakeSlidesReady) {
            outtakeSlidesRightMotor.setPower(1);
            outtakeSlidesLeftMotor.setPower(1);
        }
        else if (gamepad.right_bumper && areOuttakeSlidesReady) {
            outtakeSlidesRightMotor.setPower(-1);
            outtakeSlidesLeftMotor.setPower(-1);
        }
        else if(areOuttakeSlidesReady) {
            outtakeSlidesRightMotor.setPower(0);
            outtakeSlidesLeftMotor.setPower(0);
        }

        // Intake slides control
        if(gamepad.left_trigger > 0 && areIntakeSlidesReady) { intakeSlidesMotor.setPower(gamepad.left_trigger);}
        else if (gamepad.right_trigger > 0 && areIntakeSlidesReady) { intakeSlidesMotor.setPower(-gamepad.right_trigger);}
        else if (areIntakeSlidesReady) { intakeSlidesMotor.setPower(0); }

    }
}


