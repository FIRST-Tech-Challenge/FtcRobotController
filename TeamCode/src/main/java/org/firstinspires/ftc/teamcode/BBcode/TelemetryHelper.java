package org.firstinspires.ftc.teamcode.BBcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.text.MessageFormat;

public class TelemetryHelper {
    private OpMode _opMode;
    public TelemetryHelper(OpMode opMode)
    {
        _opMode = opMode;
    }
    //Initializes telemetry for a motor
    public void initMotorTelemetry(DcMotorEx motor, String motorName)
    {
        if (motor == null)
        {
            _opMode.telemetry.addLine("Motor " + motorName + " not found!");
            return;
        }
        //Using functions for the telemetry addData method allows for the value to be updated each time the telemetry is updated
        _opMode.telemetry.addLine(motorName)
                .addData("Pos", motor::getCurrentPosition)
                .addData("Tgt", motor::getTargetPosition)
                .addData("P", "%.2f", motor::getPower)
                .addData("V", "%.2f", motor::getVelocity);
        _opMode.telemetry.update();
    }
    //Initializes telemetry for a gamepad
    public void initGamepadTelemetry(Gamepad gamepad)
    {
        _opMode.telemetry.addLine("gamepad 1")
                .addData("X", () -> {return TFAbbr(gamepad.x);})
                .addData("Y", () -> {return TFAbbr(gamepad.y);})
                .addData("A", () -> {return TFAbbr(gamepad.a);})
                .addData("B", () -> {return TFAbbr(gamepad.b);})
                .addData("RB", () -> {return TFAbbr(gamepad.right_bumper);})
                .addData("LB", () -> {return TFAbbr(gamepad.right_bumper);});
        _opMode.telemetry.addLine()
                .addData("DPad Up", () -> {return TFAbbr(gamepad.dpad_up);})
                .addData("DPad Down", () -> {return TFAbbr(gamepad.dpad_down);})
                .addData("DPad Left", () -> {return TFAbbr(gamepad.dpad_left);})
                .addData("DPad Right", () -> {return TFAbbr(gamepad.dpad_right);});
        _opMode.telemetry.addLine()
                .addData("LS", () -> {return MessageFormat.format("'{'{0, number, #.##} , {1, number, #.##}'}'", gamepad.left_stick_x, gamepad.left_stick_y);})
                .addData("RS", () -> {return MessageFormat.format("'{'{0, number, #.##} , {1, number, #.##}'}'", gamepad.right_stick_x, gamepad.right_stick_y);})
                .addData("LT", "%.2f", () -> {return gamepad.left_trigger;})
                .addData("RT", "%.2f", () -> {return gamepad.right_trigger;});
        _opMode.telemetry.update();
    }
    //Converts boolean to T or F
    private static char TFAbbr(boolean value) {
        return value ? 'T' : 'F';
    }
}
