package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.HashMap;

public class GamePadHandler implements NKNComponent {
    enum GamepadButtons {
        LEFT_TRIGGER {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        RIGHT_TRIGGER {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        LEFT_BUMPER {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        RIGHT_BUMPER {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        DPAD_LEFT {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        DPAD_DOWN {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        DPAD_RIGHT {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        DPAD_UP {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        A {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        B {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        X {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        },
        Y {
            @Override
            boolean detect(Gamepad shortVersion) {
                return false;
            }
        };


        abstract boolean detect(Gamepad shortVersion);
    }

    private final double TRIGGERDEADZONE = 0.5;

    // Hashmap of the button to list for, and the event to trigger based on that
    // The key is the button + the name of the event
    private HashMap<String, Runnable> eventListeners;

    private Gamepad gamePad1;
    private Gamepad gamePad2;

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


}
