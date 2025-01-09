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
import org.firstinspires.ftc.teamcode.intake.IntakeSlides;
import org.firstinspires.ftc.teamcode.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.intake.IntakeElbow;
import org.firstinspires.ftc.teamcode.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.intake.IntakeClaw;
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
    boolean         wasDPadUpPressed;
    boolean         wasDPadDownPressed;
    boolean         wasDPadLeftPressed;

    public Collecting() {

        intakeSlides = new IntakeSlides();
        intakeArm = new IntakeArm();
        intakeElbow = new IntakeElbow();
        intakeWrist = new IntakeWrist();
        intakeClaw = new IntakeClaw();

        outtakeSlides = new OuttakeSlides();
        outtakeElbow = new OuttakeElbow();
        outtakeWrist = new OuttakeWrist();
        outtakeClaw = new OuttakeClaw();

        wasXPressed = false;
        wasDPadDownPressed = false;
        wasDPadUpPressed = false;
        wasDPadLeftPressed = false;

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
            logger.addLine(String.format("==> EXT OUT SLD"));
            outtakeSlides.extend(1.0);
        }
        else if (gamepad.right_bumper) {
            logger.addLine(String.format("==> RLB OUT SL"));
            outtakeSlides.rollback(1.0);
        }
        else                            {
            outtakeSlides.stop();
        }

        if(gamepad.left_trigger > 0 )                {
            logger.addLine(String.format("==> EXT IN SLD"));
            intakeSlides.extend(gamepad.left_trigger);
        }
        else if (gamepad.right_trigger > 0)          {
            logger.addLine(String.format("==> RLB IN SLD"));
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

        if(gamepad.dpad_left) {
            logger.addLine(String.format("==> SWT IN CLW : " + intakeClaw.getPosition()));
            if(!wasDPadLeftPressed){  intakeClaw.switchPosition(); }
            wasDPadLeftPressed = true;
        }
        else {
            wasDPadLeftPressed = false;
        }

        if(gamepad.dpad_up)     {
            logger.addLine(String.format("==> MUP IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadUpPressed){ intakeArm.moveUp(); }
            wasDPadUpPressed = true;
        }
        else {
            wasDPadUpPressed = false;
        }

        if(gamepad.dpad_down) {
            logger.addLine(String.format("==> MDW IN ARM : " + intakeArm.getPosition()));
            if(!wasDPadDownPressed){ intakeArm.moveDown(); }
            wasDPadDownPressed = true;
        }
        else {
            wasDPadDownPressed = false;
        }

    }
}


