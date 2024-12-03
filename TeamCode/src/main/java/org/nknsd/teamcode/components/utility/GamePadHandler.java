package org.nknsd.teamcode.components.utility;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.helperClasses.EventPair;

import java.util.ArrayList;
import java.util.TreeMap;
import java.util.concurrent.Callable;

public class GamePadHandler implements NKNComponent {
    // TreeMap of the button to list for, and the event to trigger based on that
    // The key is the button + the name of the event
    private TreeMap<String, Runnable> eventListeners = new TreeMap<String, Runnable>();
    private ArrayList<EventPair> eventListeners2 = new ArrayList<EventPair>();
    private Telemetry telemetry;
    private final double TRIGGERDEADZONE = 0.5;
    private Gamepad gamePad1;
    private Gamepad gamePad2;


    private void iterateListeners() {
//        telemetry.addData("Iterate Triggered", "Yes");
        for (EventPair eventListener : eventListeners2) {
//            telemetry.addData("Event Checked", eventListener.name);
            try {
                if (eventListener.listener.call()) {
                    eventListener.event.run();
//                    telemetry.addData("Event run", eventListener.name);
                }
            } catch (Exception e) {
                telemetry.addData("Caught an exception!! REALLY BAD!! GET DILLON!! Event Name", eventListener.name);
                telemetry.addData("Error", e);
            }
        }
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamePad1, Gamepad gamePad2) {
        this.gamePad1 = gamePad1;
        this.gamePad2 = gamePad2;
        this.telemetry = telemetry;

        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
        iterateListeners();
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
        iterateListeners();
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
        eventListeners2.forEach((n) -> telemetry.addData("Event Found", n.name));
    }

    public Gamepad getGamePad1() {
        return this.gamePad1;
    }

    public Gamepad getGamePad2() {
        return this.gamePad2;
    }

    // singular determines if the event should trigger once per key press or multiple times
    public void addListener(GamepadButtons button, int gamepadNumber, String eventName, boolean singular, Runnable event) {
        String keyName;
        if (singular) {
            keyName = button.name() + ":" + gamepadNumber + ":singular:" + eventName;
        } else {
            keyName = button.name() + ":" + gamepadNumber + ":multiple:" + eventName;
        }

        eventListeners.put(keyName, event);
    }

    public void addListener2(Callable<Boolean> listener, Runnable event, String name) {
        eventListeners2.add(new EventPair(listener, event, name));
    }

    public void removeListener(GamepadButtons button, int gamepadNumber, String eventName, boolean singular) {
        String keyName;
        if (singular) {
            keyName = button.name() + ":" + gamepadNumber + ":" + "singular" + ":" + eventName;
        } else {
            keyName = button.name() + ":" + gamepadNumber + ":" + "multiple" + ":" + eventName;
        }

        eventListeners.remove(keyName);
    }

    public void removeListener2(Callable listener, Runnable event) {
        for (EventPair eventListener : eventListeners2) {
            eventListener.isEqualTo(listener, event);
        }
    }

    public enum GamepadButtons {
        BACK {
            @Override
            public boolean detect(Gamepad gamepad){
                return (gamepad.back);
            }
        }, LEFT_TRIGGER {
            @Override
            public boolean detect(Gamepad gamepad) {
                return (gamepad.left_trigger > 0.5);
            }
        }, RIGHT_TRIGGER {
            @Override
            public boolean detect(Gamepad gamepad) {
                return (gamepad.right_trigger > 0.5);
            }
        }, LEFT_BUMPER {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.left_bumper;
            }
        }, RIGHT_BUMPER {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.right_bumper;
            }
        }, DPAD_LEFT {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.dpad_left;
            }
        }, DPAD_DOWN {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.dpad_down;
            }
        }, DPAD_RIGHT {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.dpad_right;
            }
        }, DPAD_UP {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.dpad_up;
            }
        }, A {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.a;
            }
        }, B {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.b;
            }
        }, X {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.x;
            }
        }, Y {
            @Override
            public boolean detect(Gamepad gamepad) {
                return gamepad.y;
            }
        };

        public abstract boolean detect(Gamepad gamepad);
    }

    public enum GamepadSticks {
        LEFT_JOYSTICK_X {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.left_stick_x;
            }
        },
        LEFT_JOYSTICK_Y {
            @Override
            public float getValue(Gamepad gamepad) {
                return -gamepad.left_stick_y;
            }
        },
        RIGHT_JOYSTICK_X {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.right_stick_x;
            }
        },
        RIGHT_JOYSTICK_Y {
            @Override
            public float getValue(Gamepad gamepad) {
                return -gamepad.right_stick_y;
            }
        };

        public abstract float getValue(Gamepad gamepad);
    }
}
