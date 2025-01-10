package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Intake includes */
import org.firstinspires.ftc.teamcode.intake.IntakeSlides;
import org.firstinspires.ftc.teamcode.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.intake.IntakeElbow;
import org.firstinspires.ftc.teamcode.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.intake.IntakeClaw;

/* Outtake includes */
import org.firstinspires.ftc.teamcode.outtake.OuttakeSlides;
import org.firstinspires.ftc.teamcode.outtake.OuttakeElbow;
import org.firstinspires.ftc.teamcode.outtake.OuttakeWrist;
import org.firstinspires.ftc.teamcode.outtake.OuttakeClaw;


public class Collecting {

    Telemetry       logger;

    IntakeSlides    intakeSlides;
    IntakeArm       intakeArm;
    IntakeElbow     intakeElbow;
    IntakeWrist     intakeWrist;
    IntakeClaw      intakeClaw;
    OuttakeSlides   outtakeSlides;
    OuttakeElbow    outtakeElbow;
    OuttakeWrist    outtakeWrist;
    OuttakeClaw     outtakeClaw;

    Gamepad         gamepad;
    boolean         wasXPressed;
    boolean         wasAPressed;
    boolean         wasYPressed;
    boolean         wasBPressed;
    boolean         wasDPadUpPressed;
    boolean         wasDPadDownPressed;
    boolean         wasDPadLeftPressed;
    boolean         wasDPadRightPressed;

    public Collecting() {

        intakeSlides = new IntakeSlides();
        intakeArm    = new IntakeArm();
        intakeElbow  = new IntakeElbow();
        intakeWrist  = new IntakeWrist();
        intakeClaw   = new IntakeClaw();

        outtakeSlides = new OuttakeSlides();
        outtakeElbow = new OuttakeElbow();
        outtakeWrist = new OuttakeWrist();
        outtakeClaw = new OuttakeClaw();

        wasXPressed = false;
        wasAPressed = false;
        wasYPressed = false;
        wasBPressed = false;
        wasDPadDownPressed = false;
        wasDPadUpPressed = false;
        wasDPadLeftPressed = false;
        wasDPadRightPressed = false;

    }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        logger = tm;
        logger.addLine("=== COLLECTING ===");

        intakeSlides.setHW(config, hwm, tm);
        intakeArm.setHW(config, hwm, tm);
        intakeElbow.setHW(config, hwm, tm);
        intakeWrist.setHW(config, hwm, tm);
        intakeClaw.setHW(config, hwm, tm);

        outtakeSlides.setHW(config, hwm, tm);
        outtakeElbow.setHW(config, hwm, tm);
        outtakeWrist.setHW(config, hwm, tm);
        outtakeClaw.setHW(config, hwm, tm);

        gamepad = gp;
    }

    public void move() {

        if (gamepad.left_bumper)       {
            logger.addLine("==> EXT OUT SLD");
            outtakeSlides.extend(1.0);
        }
        else if (gamepad.right_bumper) {
            logger.addLine("==> RLB OUT SLD");
            outtakeSlides.rollback(1.0);
        }
        else                            {
            outtakeSlides.stop();
        }

        if(gamepad.left_trigger > 0 )                {
            logger.addLine("==> EXT IN SLD");
            intakeSlides.extend(gamepad.left_trigger);
        }
        else if (gamepad.right_trigger > 0)          {
            logger.addLine("==> RLB IN SLD");
            intakeSlides.rollback(gamepad.right_trigger);
        }
        else                                         {
            intakeSlides.stop();
        }

        if(gamepad.x)                 {
            logger.addLine(String.format("==> SWT OUT CLW : " + outtakeClaw.getPosition()));
            if(!wasXPressed){ outtakeClaw.switchPosition(); }
            wasXPressed = true;
        }
        else {
            wasXPressed = false;
        }

        if(gamepad.y)     {
            logger.addLine(String.format("==> MDW OUT ARM : " + outtakeElbow.getPosition()));
            if(!wasYPressed){ outtakeElbow.moveDown(); }
            wasYPressed = true;
        }
        else { wasYPressed = false; }

        if(gamepad.a) {
            logger.addLine(String.format("==> MUP OUT ARM : " + outtakeElbow.getPosition()));
            if(!wasAPressed){ outtakeElbow.moveUp();}
            wasAPressed = true;
        }
        else { wasAPressed = false; }

        if(gamepad.b) {
            logger.addLine(String.format("==> CENTER OUT WRS : " + outtakeWrist.getPosition()));
            if(!wasBPressed){ outtakeWrist.setPosition(OuttakeWrist.Position.CENTER);}
            wasBPressed = true;
        }
        else { wasBPressed = false; }

        if(gamepad.dpad_left) {
            logger.addLine(String.format("==> SWT IN CLW : " + intakeClaw.getPosition()));
            if(!wasDPadLeftPressed){  intakeClaw.switchPosition(); }
            wasDPadLeftPressed = true;
        }
        else { wasDPadLeftPressed = false; }

        if(gamepad.dpad_up)     {
            logger.addLine(String.format("==> MDW IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadUpPressed){ intakeElbow.moveDown(); intakeArm.moveDown(); }
            wasDPadUpPressed = true;
        }
        else { wasDPadUpPressed = false; }

        if(gamepad.dpad_down) {
            logger.addLine(String.format("==> MUP IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadDownPressed){ intakeArm.moveUp(); intakeElbow.moveUp();}
            wasDPadDownPressed = true;
        }
        else { wasDPadDownPressed = false; }

        if(gamepad.dpad_right) {
            logger.addLine(String.format("==> CENTER IN WRS : " + intakeWrist.getPosition()));
            if(!wasDPadRightPressed){ intakeWrist.setPosition(IntakeWrist.Position.CENTER);}
            wasDPadRightPressed = true;
        }
        else { wasDPadRightPressed = false; }

        intakeWrist.turn(gamepad.left_stick_x);
        outtakeWrist.turn(gamepad.right_stick_x);

    }
}


