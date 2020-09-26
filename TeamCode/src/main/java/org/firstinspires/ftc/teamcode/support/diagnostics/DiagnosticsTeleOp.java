package org.firstinspires.ftc.teamcode.support.diagnostics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.lang.reflect.InvocationTargetException;
import java.text.SimpleDateFormat;

/**
 * Base class for diagnostic TeleOps. To create a TeleOp for new robot,
 *  extends this class in <code>org.firstinspires.ftc.teamcode</code> package
 *  and implement <code>createRobot()</code> method to create an instance of your robot.
 * @see DiagnosticsTeleOp#createRobot()
 */
public abstract class DiagnosticsTeleOp extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    private Menu menu;
    private int selectedEntryIndex;

    private EventManager eventManager1;
    private EventManager eventManager2;

    /**
     * Abstract method to be implemented in a TeleOp for each individual robot.
     * Typical implementation would look like <pre>
     *     return new MyRobot().configureLogging("Robot", Log.VERBOSE);
     * </pre>
     *
     * where <code>MyRobot</code> is a name of robot class
     * @return created robot instance
     */
    public abstract Robot2 createRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        Robot2 robot2 = createRobot();
        configuration = new Configuration(hardwareMap, robot2.getName()).configureLogging("Config", LOG_LEVEL);
        menu = new Menu();
        ToboSigma.AutoTeamColor autoColor = ToboSigma.AutoTeamColor.DIAGNOSIS;

        try {
            // configure robot and reset all hardware
            robot2.configure(configuration, telemetry, autoColor);
            if (configuration.apply()) {
                if (configuration.getLastModified()!=null) {
                    telemetry.addData("Phone adjustments", new SimpleDateFormat("MMM d, HH:mm").format(configuration.getLastModified()));
                }
            } else {
                telemetry.addData("WARNING", "Unable to load adjustments from phone or assets");
            }
            robot2.reset(false);

            // populate diagnostic menu entries
            menu.detectEntries(robot2);
            menu.detectEntries(this);

            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        resetEventManagers();
        attachMenuListeners();
        renderMenu();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try {
                eventManager1.processEvents();
                eventManager2.processEvents();
                TaskManager.processTasks();
            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
        robot2.end();
    }

    /**
     * Adjust <code>@Adjustable</code> properties for
     *  all <code>Configurable</code> components / devices defined for this robot
     */
    @MenuEntry(label = "Adjust", group="Configuration")
    public void changeAdjustments(EventManager em) {
        new Adjuster(configuration, telemetry).configureLogging("Adjuster", LOG_LEVEL).run(em);
    }

    /**
     * View <code>@Adjustable</code> properties for
     *  all <code>Configurable</code> components / devices defined for this robot
     */
    @MenuEntry(label = "View", group="Configuration")
    public void viewConfiguration(EventManager em) {
        new Adjuster(configuration, telemetry).configureLogging("Adjuster", LOG_LEVEL).show(em);
    }

    /**
     * Menu to test event handling for Gamepad1
     */
    @MenuEntry(label = "Gamepad Test")
    public void testGamepad(EventManager em) {
        GamepadListener listener = new GamepadListener();
        listener.setupTelemetry(telemetry);
        em.onStick(listener, Events.Axis.BOTH, Events.Side.LEFT, Events.Side.RIGHT);
        em.onTrigger(listener, Events.Side.LEFT, Events.Side.RIGHT);
        em.onButtonDown(listener, Button.values());
        em.onButtonUp(listener, Button.values());
        em.updateTelemetry(telemetry, 100);
    }

    private void resetEventManagers() {
        eventManager1 = new EventManager(gamepad1, true)
                .configureLogging("Gamepad1", LOG_LEVEL);
        eventManager2 = new EventManager(gamepad2, true)
                .configureLogging("Gamepad2", LOG_LEVEL);
        eventManager1.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (!source.isPressed(Button.START) || !source.isPressed(Button.BACK)) return;
                resetEventManagers();
                attachMenuListeners();
                renderMenu();
            }
        }, Button.START, Button.BACK);
    }

    private void attachMenuListeners() {
        eventManager1.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                int step = source.isPressed(Button.DPAD_UP) ? -1 : 1;
                String selectedGroup = menu.getEntries().get(selectedEntryIndex).getGroup();
                int nextIndex = selectedEntryIndex;
                while (true) {
                    nextIndex += step;
                    if (nextIndex < 0) {
                        nextIndex = menu.getEntries().size() - 1;
                    } else if (nextIndex >= menu.getEntries().size()) {
                        nextIndex = 0;
                    }
                    String group = menu.getEntries().get(nextIndex).getGroup();
                    log.verbose("U/D: index=%d, next=%d, step=%d, size=%d, group=%s, sel=%s",
                            selectedEntryIndex, nextIndex, step, menu.getEntries().size(), group, selectedGroup
                    );
                    if (nextIndex == selectedEntryIndex || group.length() == 0 || !group.equals(selectedGroup)) break;
                    DiagnosticsTeleOp.this.idle();
                }
                selectedEntryIndex = nextIndex;
                renderMenu();
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);

        eventManager1.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                int step = source.isPressed(Button.DPAD_LEFT) ? -1 : 1;
                String selectedGroup = menu.getEntries().get(selectedEntryIndex).getGroup();
                if (selectedGroup.length()==0) return;
                int nextIndex = selectedEntryIndex;
                while (true) {
                    nextIndex += step;
                    if (nextIndex < 0) {
                        nextIndex = menu.getEntries().size() - 1;
                    } else if (nextIndex >= menu.getEntries().size()) {
                        nextIndex = 0;
                    }
                    String group = menu.getEntries().get(nextIndex).getGroup();
                    log.verbose("R/L: index=%d, next=%d, step=%d, size=%d, group=%s, sel=%s",
                            selectedEntryIndex, nextIndex, step, menu.getEntries().size(), group, selectedGroup
                    );
                    if (nextIndex == selectedEntryIndex || group.equals(selectedGroup)) break;
                    DiagnosticsTeleOp.this.idle();
                }
                selectedEntryIndex = nextIndex;
                renderMenu();
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);

        eventManager1.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.START)) return;
                telemetry.clearAll();
                telemetry.addData("[Back] + [Start]", "Return to Menu").setRetained(true);
                try {
                    resetEventManagers();
                    menu.getEntries().get(selectedEntryIndex).invoke(eventManager1, eventManager2);
                } catch (IllegalAccessException E) {
                    telemetry.addData("Illegal Access", "@MenuEntry method is not public?");
                    handleException(E);
                } catch (InvocationTargetException E) {
                    telemetry.addData("@MenuEntry invocation failed", "see log for details");
                    handleException(E.getCause()==null ? E : E.getCause());
                }
                telemetry.update();
            }
        }, Button.A);
    }

    private void renderMenu() {
        Menu.Entry selectedEntry = menu.getEntries().get(selectedEntryIndex);
        telemetry.clearAll();
        telemetry.addLine()
                .addData("DPAD", "navigate")
                .addData("[A]", "enter");
        if (selectedEntry.getGroup().length()==0) {
            telemetry.addLine().addData(selectedEntry.getLabel(), "(no submenu)");
        } else {
            telemetry.addLine().addData(selectedEntry.getGroup(), selectedEntry.getLabel());
        }
        telemetry.update();
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
