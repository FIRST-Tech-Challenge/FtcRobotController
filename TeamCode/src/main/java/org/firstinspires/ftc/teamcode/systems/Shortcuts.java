package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.function.Consumer;

public class Shortcuts {
    private final BaseRobot robot;
    private final Map<Settings.GamepadButton, List<Action>> shortcuts = new HashMap<>();
    private final Map<Settings.GamepadButton, Boolean> buttonStates = new HashMap<>();
    private boolean isExecutingShortcut = false;

    public Shortcuts(BaseRobot robot) {
        this.robot = robot;
        // Initialize button states
        for (Settings.GamepadButton button : Settings.GamepadButton.values()) {
            buttonStates.put(button, false);
        }
    }

    public static class Action {
        private final Consumer<BaseRobot> action;
        private final long delayAfter;

        public Action(Consumer<BaseRobot> action, long delayAfter) {
            this.action = action;
            this.delayAfter = delayAfter;
        }

        public void execute(BaseRobot robot) {
            action.accept(robot);
            if (delayAfter > 0) {
                try {
                    Thread.sleep(delayAfter);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public void addShortcut(Settings.GamepadButton button, List<Action> actions) {
        shortcuts.put(button, actions);
    }

    public void handleInput(Settings.GamepadButton button, boolean pressed) {
        if (isExecutingShortcut)
            return;

        boolean wasPressed = buttonStates.get(button);
        buttonStates.put(button, pressed);

        if (pressed && !wasPressed && shortcuts.containsKey(button)) {
            executeShortcut(button);
        }
    }

    private void executeShortcut(Settings.GamepadButton button) {
        if (!shortcuts.containsKey(button))
            return;

        isExecutingShortcut = true;
        CompletableFuture.runAsync(() -> {
            try {
                for (Action action : shortcuts.get(button)) {
                    action.execute(robot);
                }
            } finally {
                isExecutingShortcut = false;
            }
        });
    }

    // Predefined action builders
    public static Action moveForward(int counts) {
        return new Action(robot -> robot.odometry.moveCounts("forward", counts), 0);
    }

    public static Action moveBackward(int counts) {
        return new Action(robot -> robot.odometry.moveCounts("backward", counts), 0);
    }

    public static Action moveLeft(int counts) {
        return new Action(robot -> robot.odometry.moveCounts("left", counts), 0);
    }

    public static Action moveRight(int counts) {
        return new Action(robot -> robot.odometry.moveCounts("right", counts), 0);
    }

    public static Action rotate(int counts) {
        return new Action(robot -> robot.odometry.moveCounts("tright", counts), 0);
    }

    public static Action pause(long ms) {
        return new Action(robot -> {
        }, ms);
    }

    public static Action extendArm() {
        return new Action(robot -> robot.arm.extensor.extend(), 500);
    }

    public static Action retractArm() {
        return new Action(robot -> robot.arm.extensor.retract(), 500);
    }

    public static Action openClaw() {
        return new Action(robot -> {
            robot.arm.claw.setLeftServo(true);
            robot.arm.claw.setRightServo(true);
        }, 250);
    }

    public static Action closeClaw() {
        return new Action(robot -> {
            robot.arm.claw.setLeftServo(false);
            robot.arm.claw.setRightServo(false);
        }, 250);
    }
}