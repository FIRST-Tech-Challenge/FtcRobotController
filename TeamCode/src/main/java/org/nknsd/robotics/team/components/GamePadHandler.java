package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class GamePadHandler implements NKNComponent {


    private Gamepad gamePad1;
    private Gamepad gamePad2;

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamePad1, Gamepad gamePad2) {
        this.gamePad1 = gamePad1;
        this.gamePad2 = gamePad2;

        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "GamePadHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    private String buildControllerString(Gamepad gamePad) {
        String g1String = "[ l_x" + gamePad.left_stick_x + ", l_y" + gamePad.left_stick_y;
        if (gamePad.left_stick_button) {
            g1String = g1String + "l_s_b ";
        }
        g1String = g1String + " r_x" + gamePad.right_stick_x + ", r_y" + gamePad.right_stick_y;
        if (gamePad.right_stick_button) {
            g1String = g1String + "r_s_b ";
        }
        if (gamePad.a) {
            g1String = g1String + " a ";
        }
        if (gamePad.b) {
            g1String = g1String + "b ";
        }
        if (gamePad.x) {
            g1String = g1String + "x ";
        }
        if (gamePad.y) {
            g1String = g1String + "y ";
        }
        if (gamePad.dpad_left) {
            g1String = g1String + " dpad_l ";
        }
        if (gamePad.dpad_right) {
            g1String = g1String + " dpad_r ";
        }
        if (gamePad.dpad_up) {
            g1String = g1String + " dpad_u ";
        }
        if (gamePad.dpad_down) {
            g1String = g1String + " dpad_d ";
        }
        if (gamePad.left_bumper) {
            g1String = g1String + " l_bumper ";
        }
        if (gamePad.right_bumper) {
            g1String = g1String + " r_bumper ]";
        }
        return g1String;
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        String gp1String = buildControllerString(gamePad1);
        String gp2String = buildControllerString(gamePad2);
        telemetry.addData("gPad1", gp1String);
        telemetry.addData("gPad2", gp2String);
    }

    public Gamepad getGamePad1() {
        return this.gamePad1;
    }

    public Gamepad getGamePad2() {
        return this.gamePad2;
    }
}
