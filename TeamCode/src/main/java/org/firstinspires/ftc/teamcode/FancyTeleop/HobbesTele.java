package org.firstinspires.ftc.teamcode.FancyTeleop;

import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_OUT_FULL;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_OUT_SOME;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_SPEED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_INTAKE_FLAT;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_ARM_ABOVE_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_ARM_DEPOSIT;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_ARM_SPECIMEN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_IN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_OUT_TOP_SAMPLE;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_OUT_TOP_SPECIMEN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_OUT_TOP_SPECIMEN_DOWN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_SPECIMEN_PICKUP;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_WRIST_DEPOSIT;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_WRIST_SPECIMEN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.SLIDES_WRIST_TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

@TeleOp
public class HobbesTele extends OpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Hobbes hob = null;
    Map<String, HobbesState> macros = new HashMap<>();

    @Override
    public void init() {
        macros.put("EXTENDO_BEFORE_PICKUP", new HobbesState(EXTENDO_OUT_SOME, null, null, null, null, INTAKE_POWER, null, null, new LinkedState("EXTENDO_BEFORE_PICKUP2",2000)));
        macros.put("EXTENDO_BEFORE_PICKUP2", new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, INTAKE_POWER, null, null, null));

        macros.put("FULL_IN" ,new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, null));


        macros.put("FULL_TRANSFER" ,new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState("TRANSFER_ON", 3000)));
        macros.put("TRANSFER_ON", new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState("TRANSFER_CLOSED", 1000)));
        macros.put("TRANSFER_CLOSED", new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN, null));

        macros.put("SLIDES_DOWN", new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN, null));


        macros.put("SLIDES_DEPOSIT", new HobbesState(null, null, null, null, null, null, null, SLIDES_OUT_TOP_SAMPLE, new LinkedState("SLIDES_DEPOSIT2", 1200)));
        macros.put("SLIDES_DEPOSIT2", new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null, null));


        macros.put("OPEN_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null, null));
        macros.put("CLOSE_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, null));

      //  macros.put("SLIDES_SPECIMEN_PICKUP", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_OPEN, SLIDES_SPECIMEN_PICKUP, null));
   //     macros.put("SPECIMEN_CLOSE_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, new LinkedState("SLIDES_SPECIMEN_ABOVE_DEPOSIT", 200)));

     //   macros.put("SLIDES_SPECIMEN_ABOVE_DEPOSIT", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_CLOSED, SLIDES_OUT_TOP_SPECIMEN, null));

     //   macros.put("SLIDES_SPECIMEN_DEPOSIT", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_CLOSED, SLIDES_OUT_TOP_SPECIMEN_DOWN, new LinkedState("OPEN_CLAW", 500)));

        hob = new Hobbes();
        hob.init(hardwareMap);
        hob.setMacros(macros);
    }
    @Override
    public void start() {
       hob.runMacro("FULL_IN");
    }

    @Override
    public void loop() {
        // movement
        hob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad2.left_stick_y != 0) {
            hob.slidesController.driveSlides(gamepad2.left_stick_y);
        }
        hob.servosController.incrementExtendo(gamepad2.right_stick_y * EXTENDO_SPEED);

        // before pickup macro


        if (gamepad2.left_stick_button || gamepad2.right_stick_button) {
            hob.cancelMacros();
        }

        if (gamepad2.b && !lastGamepad2.b) {
            hob.runMacro("EXTENDO_BEFORE_PICKUP");
        }

        // full transfer macro
        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) {
            hob.runMacro("FULL_TRANSFER");
        }

        // deposit macro (assumes in transfer mode beforehand)
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) {
            hob.runMacro("SLIDES_DEPOSIT");
        }

        if (gamepad2.left_bumper && !lastGamepad2.left_bumper) {
            hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);
        }

        if (gamepad2.dpad_down && !lastGamepad2.dpad_down) {
            hob.runMacro("SLIDES_DOWN");

        }



        hob.tick();
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }
    @Override
    public void stop() {

    }


}
