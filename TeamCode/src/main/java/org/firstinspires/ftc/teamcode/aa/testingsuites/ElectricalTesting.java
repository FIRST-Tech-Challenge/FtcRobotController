package org.firstinspires.ftc.teamcode.aa.testingsuites;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ConfigMan;
import org.firstinspires.ftc.teamcode.util.Configurable;

import java.util.HashMap;

public class ElectricalTesting extends LinearOpMode implements ConfigMan, Configurable {


    // Gamepad controls
    double rx;
    double ry;
    boolean buttonA;
    boolean buttonB;
    boolean buttonX;
    boolean buttonY;
    boolean dpadUp;
    boolean dpadDown;
    boolean dpadLeft;
    boolean dpadRight;
    double rt;
    double lt;
    double lx;
    double ly;
    boolean lb;
    boolean rb;
    boolean start;
    boolean back;

    String schemaName;
    Telemetry telemetries;

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor motor5;
    DcMotor motor6;
    DcMotor motor7;
    DcMotor motor8;


    public ElectricalTesting(String schemaName, Telemetry telemetries) {
        this.schemaName = schemaName;
        this.telemetries = telemetries;
    }
    @Override
    public void runOpMode(){
        waitForStart();
    }

    public void updateGamepad() {
        rx = gamepad1.right_stick_x;
        ry = gamepad1.right_stick_y;
        buttonA = gamepad1.a;
        buttonB = gamepad1.b;
        buttonX = gamepad1.x;
        buttonY = gamepad1.y;
        dpadUp = gamepad1.dpad_up;
        dpadDown = gamepad1.dpad_down;
        dpadRight = gamepad1.dpad_right;
        dpadLeft = gamepad1.dpad_left;
        rt = gamepad1.left_trigger;
        lt = gamepad1.right_trigger;

        lx = gamepad1.left_stick_x;
        ly = gamepad1.left_stick_y;

        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;

        back = gamepad1.back;
        start = gamepad1.start;

    }

    // ConfigMans
    @Override
    public void resetConfiguration() {

    }

    @Override
    public void setConfig(HashMap<String, String> config) {

    }

    @Override
    public String getConfig(String key) {
        return null;
    }

    @Override
    public String addConfigData(String key, String data) {
        return null;
    }

    @Override
    public void replaceConfigKey(String key, String data) throws Exception {

    }
}
