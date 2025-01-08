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

        if(intakeSlides.length()           == 0) { logger.addLine("Missing intake slides motor configuration") ;             isIntakeReady = false;  }
        if(intakeArmLeftPitch.length()     == 0) { logger.addLine("Missing intake arm left pitch servo configuration") ;     isIntakeReady = false;  }
        if(intakeArmRightPitch.length()    == 0) { logger.addLine("Missing intake arm right pitch servo configuration") ;    isIntakeReady = false;  }
        if(intakeElbowPitch.length()       == 0) { logger.addLine("Missing intake elbow pitch servo configuration") ;        isIntakeReady = false;  }
        if(intakeWristRoll.length()        == 0) { logger.addLine("Missing intake wrist roll servo configuration") ;         isIntakeReady = false;  }
        if(intakeClaw.length()             == 0) { logger.addLine("Missing intake claw servo configuration") ;               isIntakeReady = false;  }
        if(outtakeLeftSlides.length()      == 0) { logger.addLine("Missing outtake slides left motor configuration") ;       isOuttakeReady = false; }
        if(outtakeRightSlides.length()     == 0) { logger.addLine("Missing outtake slides right motor configuration") ;      isOuttakeReady = false; }
        if(outtakeElbowLeftPitch.length()  == 0) { logger.addLine("Missing outtake elbow left pitch servo configuration") ;  isOuttakeReady = false;  }
        if(outtakeElbowRightPitch.length() == 0) { logger.addLine("Missing outtake elbow right pitch servo configuration") ; isOuttakeReady = false;  }
        if(outtakeWristRoll.length()       == 0) { logger.addLine("Missing outtake wrist roll servo configuration") ;        isOuttakeReady = false;  }
        if(outtakeClaw.length()            == 0) { logger.addLine("Missing outtake claw servo configuration") ;              isOuttakeReady = false;  }

        if (isIntakeReady) {
            intakeSlidesMotor = hwm.tryGet(DcMotor.class, intakeSlides);
            intakeArmLeftPitchServo = hwm.tryGet(Servo.class, intakeArmLeftPitch);
            intakeArmRightPitchServo = hwm.tryGet(Servo.class, intakeArmRightPitch);
            intakeElbowPitchServo = hwm.tryGet(Servo.class, intakeElbowPitch);
            intakeWristRollServo = hwm.tryGet(Servo.class, intakeWristRoll);
            intakeClawServo = hwm.tryGet(Servo.class, intakeClaw);
        }

        if(isOuttakeReady) {
            outtakeSlidesLeftMotor = hwm.tryGet(DcMotor.class,outtakeLeftSlides);
            outtakeSlidesRightMotor = hwm.tryGet(DcMotor.class,outtakeRightSlides);
            outtakeElbowRightPitchServo = hwm.tryGet(Servo.class,outtakeElbowRightPitch);
            outtakeElbowLeftPitchServo = hwm.tryGet(Servo.class,outtakeElbowLeftPitch);
            outtakeWristRollServo = hwm.tryGet(Servo.class,outtakeWristRoll);
            outtakeClawServo = hwm.tryGet(Servo.class,outtakeClaw);

        }

        if (isIntakeReady) {
            if (config.INTAKE_SLIDES_REVERSE() && intakeSlidesMotor != null) {
                intakeSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

        }
        if(isOuttakeReady) {
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

        if(isIntakeReady) { logger.addLine("Intake is ready"); }
        else { logger.addLine("Intake is not ready"); }

        if(isOuttakeReady) { logger.addLine("Outtake is ready"); }
        else { logger.addLine("Outtake is not ready"); }
    }

    public void move() {

        // Outtake slides control
        if (gamepad.left_bumper && outtakeSlidesRightMotor != null && outtakeSlidesLeftMotor != null) {
            outtakeSlidesRightMotor.setPower(1);
            outtakeSlidesLeftMotor.setPower(1);
        }
        else if (gamepad.right_bumper && outtakeSlidesRightMotor != null && outtakeSlidesLeftMotor != null) {
            outtakeSlidesRightMotor.setPower(-1);
            outtakeSlidesLeftMotor.setPower(-1);
        }
        else if(outtakeSlidesRightMotor != null && outtakeSlidesLeftMotor != null) {
            outtakeSlidesRightMotor.setPower(0);
            outtakeSlidesLeftMotor.setPower(0);
        }

        // Intake slides control
        if(gamepad.left_trigger > 0 && intakeSlidesMotor != null) { intakeSlidesMotor.setPower(gamepad.left_trigger);}
        else if (gamepad.right_trigger > 0 && intakeSlidesMotor != null) { intakeSlidesMotor.setPower(-gamepad.right_trigger);}
        else if (intakeSlidesMotor != null) { intakeSlidesMotor.setPower(0); }

    }
}


