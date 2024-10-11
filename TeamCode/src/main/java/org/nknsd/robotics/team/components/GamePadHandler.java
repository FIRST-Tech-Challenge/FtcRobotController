package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.SortedMap;
import java.util.TreeMap;

public class GamePadHandler implements NKNComponent {
    // TreeMap of the button to list for, and the event to trigger based on that
    // The key is the button + the name of the event
    private TreeMap<String, Runnable> eventListeners = new TreeMap<String, Runnable>();
    private final double TRIGGERDEADZONE = 0.5;

    // Iterates through the eventListeners tree map to call the runnables on a given button
    private void activateListener(String button, int gamepadNumber) {
        String searchKey = button + ":" + gamepadNumber;

        for (Runnable r : eventListeners.subMap(searchKey, searchKey + ";").values()) {
            r.run();
        }
    }
    private Gamepad gamePad1;
    private Gamepad gamePad2;

    // Iterates through the buttons of a gamepad and activates the listeners of any functions that are attached
    public void checkButtons(Gamepad gamepad, int gamepadNumber) {
        for (GamepadButtons button : GamepadButtons.values()) {
            if (button.detect(gamepad)) {
                activateListener(button.name(), gamepadNumber);
            }
        }
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamePad1, Gamepad gamePad2) {
        this.gamePad1 = gamePad1;
        this.gamePad2 = gamePad2;

        return false;
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
        String g1String = "[l_x" + gamePad.left_stick_x + ":l_y" + gamePad.left_stick_y;
        g1String = g1String + ":r_x" + gamePad.right_stick_x + ":r_y" + gamePad.right_stick_y;
        if (gamePad.left_stick_button) {
            g1String = g1String + ":l_s_b";
        }
        if (gamePad.right_stick_button) {
            g1String = g1String + ":r_s_b";
        }
        if (gamePad.a) {
            g1String = g1String + ":a";
        }
        if (gamePad.b) {
            g1String = g1String + ":b";
        }
        if (gamePad.x) {
            g1String = g1String + ":x";
        }
        if (gamePad.y) {
            g1String = g1String + ":y";
        }
        if (gamePad.dpad_left) {
            g1String = g1String + ":dpad_l";
        }
        if (gamePad.dpad_right) {
            g1String = g1String + ":dpad_r";
        }
        if (gamePad.dpad_up) {
            g1String = g1String + ":dpad_u";
        }
        if (gamePad.dpad_down) {
            g1String = g1String + ":dpad_d";
        }
        if (gamePad.left_bumper) {
            g1String = g1String + ":l_bumper";
        }
        if (gamePad.right_bumper) {
            g1String = g1String + ":r_bumper";
        }
        if (gamePad.left_trigger > TRIGGERDEADZONE) {
            g1String = g1String + ":l_trigger";
        }
        if (gamePad.right_trigger > TRIGGERDEADZONE) {
            g1String = g1String + ":r_trigger";
        }
        g1String = g1String + "]";
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

    public void addListener(GamepadButtons button, int gamepadNumber, String eventName, Runnable event) {
        String keyName = button.name() + ":" + gamepadNumber + ":" + eventName;
        eventListeners.put(keyName, event);
    }

    public void removeListener(GamepadButtons button, int gamepadNumber, String eventName) {
        String keyName = button.name() + ":" + gamepadNumber + ":" + eventName;
        eventListeners.remove(keyName);
    }

    enum GamepadButtons {
        LEFT_TRIGGER {
            @Override
            boolean detect(Gamepad gamepad) {
                return (gamepad.left_trigger > 0.5);
            }
        }, RIGHT_TRIGGER {
            @Override
            boolean detect(Gamepad gamepad) {
                return (gamepad.right_trigger > 0.5);
            }
        }, LEFT_BUMPER {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.left_bumper;
            }
        }, RIGHT_BUMPER {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.right_bumper;
            }
        }, DPAD_LEFT {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.dpad_left;
            }
        }, DPAD_DOWN {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.dpad_down;
            }
        }, DPAD_RIGHT {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.dpad_right;
            }
        }, DPAD_UP {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.dpad_up;
            }
        }, A {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.a;
            }
        }, B {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.b;
            }
        }, X {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.x;
            }
        }, Y {
            @Override
            boolean detect(Gamepad gamepad) {
                return gamepad.y;
            }
        };

        abstract boolean detect(Gamepad gamepad);
    }
}
