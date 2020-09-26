package org.firstinspires.ftc.teamcode.support.diagnostics;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;

import java.util.EnumSet;

public class GamepadListener extends Events.Listener {
    private EnumSet<Button> buttons = EnumSet.noneOf(Button.class);
    private float leftTrigger = 0;
    private float rightTrigger = 0;
    private float leftStickX = 0;
    private float leftStickY = 0;
    private float rightStickX = 0;
    private float rightStickY = 0;

    @Override
    public void buttonDown(EventManager source, Button button) {
        buttons.add(button);
    }

    @Override
    public void buttonUp(EventManager source, Button button) {
        buttons.remove(button);
    }

    @Override
    public void triggerMoved(EventManager source, Events.Side side, float current, float change) {
        if (side== Events.Side.LEFT) {
            leftTrigger = current;
        } else {
            rightTrigger = current;
        }
    }

    @Override
    public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                           float currentY, float changeY) {
        if (side== Events.Side.LEFT) {
            leftStickX = currentX;
            leftStickY = currentY;
        } else {
            rightStickX = currentX;
            rightStickY = currentY;
        }
    }

    public void setupTelemetry(Telemetry telemetry) {
        telemetry.addLine().addData("(LS)", new Func<String>() {
            @Override
            public String value() {
                return String.format("%+.2f / %+.2f", leftStickX, leftStickY);
            }
        }).addData("(RS)", new Func<String>() {
            @Override
            public String value() {
                return String.format("%+.2f / %+.2f", rightStickX, rightStickY);
            }
        });
        telemetry.addLine().addData("[LT]", "%.2f", new Func<Float>() {
            @Override
            public Float value() { return leftTrigger; }
        }).addData("[RT]", "%.2f", new Func<Float>() {
            @Override
            public Float value() { return rightTrigger; }
        });
        telemetry.addData("Buttons", new Func<String>() {
            @Override
            public String value() {
                if (buttons.isEmpty()) return "";
                StringBuilder sb = new StringBuilder();
                for (Button button : Button.values()) {
                    if (!buttons.contains(button)) continue;
                    // pretty print button name
                    String name = button.name();
                    if (button == Button.LEFT_BUMPER) {
                        name = "LB";
                    } else if (button == Button.RIGHT_BUMPER) {
                        name = "RB";
                    } else if (button == Button.START || button == Button.BACK) {
                        name = name.toLowerCase();
                    } else if (name.startsWith("DPAD_")) {
                        name = name.substring(5).toLowerCase();
                    }
                    if (sb.length()>0) sb.append(' ');
                    sb.append('[').append(name).append(']');
                }
                return  sb.toString();
            }
        });
    }
}
