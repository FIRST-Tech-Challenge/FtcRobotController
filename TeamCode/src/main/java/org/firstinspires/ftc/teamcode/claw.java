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
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.Motor;
import org.firstinspires.ftc.teamcode.configurations.ServoMotor;

public class Claw {

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

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        isIntakeReady = true;
        isOuttakeReady = true;

        logger = tm;

        Motor intakeSlides  = config.getMotor("intake-slides");
        Motor outtakeLeftSlides = config.getMotor("outtake-left-slides");
        Motor outtakeRightSlides = config.getMotor("outtake-right-slides");

        ServoMotor intakeArmLeftPitch = config.getServo("intake-arm-left-pitch");
        ServoMotor intakeArmRightPitch = config.getServo("intake-arm-right-pitch");
        ServoMotor intakeElbowPitch = config.getServo("intake-elbow-pitch");
        ServoMotor intakeWristRoll = config.getServo("intake-wrist-roll");
        ServoMotor intakeClaw = config.getServo("intake-claw");
        ServoMotor outtakeElbowRightPitch = config.getServo("outtake-elbow-right-pitch");
        ServoMotor outtakeElbowLeftPitch = config.getServo("outtake-elbow-right-pitch");
        ServoMotor outtakeWristRoll = config.getServo("outtake-wrist-roll");
        ServoMotor outtakeClaw = config.getServo("outtake-claw");

        if(intakeSlides           == null) { tm.addLine("Missing intake slides motor configuration") ;             isIntakeReady = false;  }
        if(intakeArmLeftPitch     == null) { tm.addLine("Missing intake arm left pitch servo configuration") ;     isIntakeReady = false;  }
        if(intakeArmRightPitch    == null) { tm.addLine("Missing intake arm right pitch servo configuration") ;    isIntakeReady = false;  }
        if(intakeElbowPitch       == null) { tm.addLine("Missing intake elbow pitch servo configuration") ;        isIntakeReady = false;  }
        if(intakeWristRoll        == null) { tm.addLine("Missing intake wrist roll servo configuration") ;         isIntakeReady = false;  }
        if(intakeClaw             == null) { tm.addLine("Missing intake claw servo configuration") ;               isIntakeReady = false;  }
        if(outtakeLeftSlides      == null) { tm.addLine("Missing outtake slides left motor configuration") ;       isOuttakeReady = false; }
        if(outtakeRightSlides     == null) { tm.addLine("Missing outtake slides right motor configuration") ;      isOuttakeReady = false; }
        if(outtakeElbowLeftPitch  == null) { tm.addLine("Missing outtake elbow left pitch servo configuration") ;  isOuttakeReady = false;  }
        if(outtakeElbowRightPitch == null) { tm.addLine("Missing outtake elbow right pitch servo configuration") ; isOuttakeReady = false;  }
        if(outtakeWristRoll       == null) { tm.addLine("Missing outtake wrist roll servo configuration") ;        isOuttakeReady = false;  }
        if(outtakeClaw            == null) { tm.addLine("Missing outtake claw servo configuration") ;              isOuttakeReady = false;  }

        if (isIntakeReady) {
            intakeSlidesMotor = hwm.tryGet(DcMotor.class, intakeSlides.getName());
            intakeArmLeftPitchServo = hwm.tryGet(Servo.class, intakeArmLeftPitch.getName());
            intakeArmRightPitchServo = hwm.tryGet(Servo.class, intakeArmRightPitch.getName());
            intakeElbowPitchServo = hwm.tryGet(Servo.class, intakeElbowPitch.getName());
            intakeWristRollServo = hwm.tryGet(Servo.class, intakeWristRoll.getName());
            intakeClawServo = hwm.tryGet(Servo.class, intakeClaw.getName());
        }

        if(isOuttakeReady) {
            outtakeSlidesLeftMotor = hwm.tryGet(DcMotor.class,outtakeLeftSlides.getName());
            outtakeSlidesRightMotor = hwm.tryGet(DcMotor.class,outtakeRightSlides.getName());
            outtakeElbowRightPitchServo = hwm.tryGet(Servo.class,outtakeElbowRightPitch.getName());
            outtakeElbowLeftPitchServo = hwm.tryGet(Servo.class,outtakeElbowLeftPitch.getName());
            outtakeWristRollServo = hwm.tryGet(Servo.class,outtakeWristRoll.getName());
            outtakeClawServo = hwm.tryGet(Servo.class,outtakeClaw.getName());

        }

        if (isIntakeReady) {
            if (intakeSlides.getReverse() && intakeSlidesMotor != null) {
                intakeSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
            if(intakeSlidesMotor != null) {intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

        }
        if(isOuttakeReady) {
            if (outtakeLeftSlides.getReverse() && outtakeSlidesLeftMotor != null) {
                outtakeSlidesLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (outtakeRightSlides.getReverse() && outtakeSlidesRightMotor != null) {
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


