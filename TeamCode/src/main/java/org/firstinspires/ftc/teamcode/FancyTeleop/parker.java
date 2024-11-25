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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;

@Autonomous
public class parker extends LinearOpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    //List<Gamepad> gamepadHistory1 = new ArrayList<>(), gamepadHistory2 = new ArrayList<>();
    Hobbes hob = null;
    Map<String, HobbesState> macros = new HashMap<>();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        macros.put("EXTEND", new HobbesState(EXTENDO_OUT_SOME+0.1, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null));

        // DEFINE AND INIT ROBOT
        hob = new Hobbes();
        hob.init(hardwareMap);
        // SET MACROS TO TELEOP MACROS
        hob.setMacros(macros);
        waitForStart();
        hob.setup();
        hob.runMacro("EXTEND");
        timer.reset();
        while (timer.milliseconds() < 5000) hob.tick();

    }


}
