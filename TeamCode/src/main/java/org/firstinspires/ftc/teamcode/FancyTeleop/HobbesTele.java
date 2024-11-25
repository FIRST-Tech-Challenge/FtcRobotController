package org.firstinspires.ftc.teamcode.FancyTeleop;

import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_SPEED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_ARM_UP;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_OUT_FULL;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_OUT_SOME;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_SPEED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_INTAKE_FLAT;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_SPEED;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.EXTENDO_WRIST_UP;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.FancyTeleop.HobbesConstants.INTAKE_REVERSE;
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

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;

@TeleOp
public class HobbesTele extends OpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    //List<Gamepad> gamepadHistory1 = new ArrayList<>(), gamepadHistory2 = new ArrayList<>();
    Hobbes hob = null;
    Map<String, HobbesState> macros = new HashMap<>();

    @Override
    public void init() {
        macros.put("EXTENDO_BEFORE_PICKUP", new HobbesState(EXTENDO_OUT_SOME, null, null, null, null, INTAKE_POWER, null, null, null));
        macros.put("EXTENDO_ARM_WRIST_FLAT", new HobbesState(null, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null));
        macros.put("EXTENDO_ARM_WRIST_UP", new HobbesState(null, EXTENDO_ARM_UP, EXTENDO_WRIST_UP, null, null, INTAKE_OFF, null, null, null));
        macros.put("EXTENDO_ARM_WRIST_ANGLED", new HobbesState(null, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null));

        macros.put("FULL_IN" ,new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, null));

        macros.put("FULL_TRANSFER" ,new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState("TRANSFER_WRIST_UP", 600)));
        macros.put("TRANSFER_WRIST_UP" ,new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState("TRANSFER_ON", 200)));
        macros.put("TRANSFER_ON", new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState("TRANSFER_CLOSED", 250)));
        macros.put("TRANSFER_CLOSED", new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN, null));

        macros.put("SLIDES_DOWN", new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN, null));

        macros.put("SLIDES_DEPOSIT", new HobbesState(null, null, null, null, null, null, null, SLIDES_OUT_TOP_SAMPLE, new LinkedState("SLIDES_DEPOSIT2", 400)));
        macros.put("SLIDES_DEPOSIT2", new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null, null));

        macros.put("OPEN_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null, null));
        macros.put("CLOSE_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, null));

  //  macros.put("SLIDES_SPECIMEN_PICKUP", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_OPEN, SLIDES_SPECIMEN_PICKUP, null));
//     macros.put("SPECIMEN_CLOSE_CLAW", new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, new LinkedState("SLIDES_SPECIMEN_ABOVE_DEPOSIT", 200)));

 //   macros.put("SLIDES_SPECIMEN_ABOVE_DEPOSIT", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_CLOSED, SLIDES_OUT_TOP_SPECIMEN, null));

 //   macros.put("SLIDES_SPECIMEN_DEPOSIT", new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN, SLIDES_WRIST_SPECIMEN, null, CLAW_CLOSED, SLIDES_OUT_TOP_SPECIMEN_DOWN, new LinkedState("OPEN_CLAW", 500)));

        // DEFINE AND INIT ROBOT
        hob = new Hobbes();
        hob.init(hardwareMap);
        // SET MACROS TO TELEOP MACROS
        hob.setMacros(macros);
    }
    @Override
    public void start() {
        // RUN EVERYTHING TO START POSITIONS
        hob.setup();
    }

    @Override
    public void loop() {
        // P1: MOTION
        if (gamepad2.start || gamepad1.start) return;
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) hob.motorDriveXYVectors(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else hob.motorDriveXYVectors(0.3 * -gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);

        // P1: INTAKE
        if (gamepad1.a) hob.servosController.intake(INTAKE_POWER);
        else if (gamepad1.b) hob.servosController.intake(INTAKE_REVERSE);
        else if (gamepad1.right_trigger > 0) hob.servosController.intake(INTAKE_POWER * gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0) hob.servosController.intake(INTAKE_REVERSE *gamepad1.left_trigger);
        else hob.servosController.intake(INTAKE_OFF);

        // P2: SLIDES MOTION
        if (gamepad2.right_stick_y != 0) hob.slidesController.driveSlides(-gamepad2.right_stick_y);

        // P2: EXTENDO MOTION
        hob.servosController.incrementExtendo(-gamepad2.left_stick_y * EXTENDO_SPEED);

        // P2: FLAT ON GROUND
        if (gamepad2.b && !lastGamepad2.b) hob.runMacro("EXTENDO_ARM_WRIST_FLAT");

        // P2: UP BUT LOW
        if (gamepad2.a && !lastGamepad2.a) hob.runMacro("EXTENDO_ARM_WRIST_UP");

        // P2: ANGLED ON GROUND
        if (gamepad2.x && !lastGamepad2.x) hob.runMacro("EXTENDO_ARM_WRIST_ANGLED");

        // P2: MANUAL EXTENDO ARM ARTICULATION
        if (gamepad2.left_trigger > 0) hob.servosController.incrementArmWrist(gamepad2.left_trigger * EXTENDO_ARM_SPEED, 0);

        if (gamepad2.right_trigger > 0) hob.servosController.incrementArmWrist(gamepad2.right_trigger * -EXTENDO_ARM_SPEED, 0);

        // P2: MANUAL EXTENDO WRIST ARTICULATION
        if (gamepad2.right_bumper) hob.servosController.incrementArmWrist(0, EXTENDO_WRIST_SPEED);
        if (gamepad2.left_bumper) hob.servosController.incrementArmWrist(0, -EXTENDO_WRIST_SPEED);

        // P2: TRANSFER MACRO
        if (gamepad2.y && !lastGamepad2.y) hob.runMacro("FULL_TRANSFER");


        // P2: RUN TO DEPOSIT
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) hob.runMacro("SLIDES_DEPOSIT");

        // P2: TOGGLE CLAW
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);

        // P2: WRIST RE-ZEROER
        if (gamepad2.back) hob.extendoWristRezeroOffset = hob.servosController.extendoWristPos - EXTENDO_WRIST_INTAKE_FLAT;
        if (gamepad2.left_stick_button) hob.extendoWristRezeroOffset = 0;

        // P2: SLIDES DOWN, ARM ABOVE SAMPLE
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro("SLIDES_DOWN");


        // TICK ROBOT
        hob.tick();

        // REFRESH LAST GAMEPAD STATE
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }
    @Override
    public void stop() {

    }


}
