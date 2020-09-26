package org.firstinspires.ftc.teamcode.support.diagnostics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.hardware.Reflection;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Menu entry handler for <code>DiagnosticsTeleOp</code> that displays
 *  and adjusts all <code>@Adjustable</code> properties for <code>Configurable</code>
 *  components or devices defined in given robot configuration
 */
public class Adjuster extends Logger<Adjuster> {
    private Configuration cfg;
    private Telemetry telemetry;

    private List<String> deviceNames;
    private int deviceIndex = 0;
    private Configurable selectedDevice;

    private Map<String, Adjustable> deviceSettings;
    private int settingIndex = 0;
    private Map.Entry<String, Adjustable> selectedSetting;

    public Adjuster(Configuration configuration, Telemetry telemetry) {
        this.cfg = configuration;
        this.telemetry = telemetry;

        deviceNames = new ArrayList<String>(cfg.getComponentNames());
        Collections.sort(deviceNames);
    }

    public void show(EventManager em) {
        if (deviceNames.isEmpty()) {
            telemetry.addData("No adjustable devices found", "");
            telemetry.update();
            return;
        }

        for (String deviceName : deviceNames) {
            Telemetry.Line line = telemetry.addLine(deviceName + ": ");
            selectedDevice = cfg.get(deviceName);
            deviceSettings = cfg.getAdjustableProperties(selectedDevice);
            for (String property : deviceSettings.keySet()) {
                line.addData(property, "%.3f", getProperty(selectedDevice, property));
            }
        }

        if (cfg.getLastModified()!=null) {
            telemetry.addLine().addData("Phone adjustments", new SimpleDateFormat("MMM d, HH:mm").format(cfg.getLastModified()));
            telemetry.addLine().addData("[X] + [Y]", "Delete (Revert to Defaults)");
        }
        telemetry.update();

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (!source.isPressed(Button.X) || !source.isPressed(Button.Y) || cfg.getLastModified()==null) return;
                cfg.delete();
                cfg.apply();
                telemetry.addLine().addData("Deleted", "Reverted to Defaults");
                show(source);
            }
        }, Button.X, Button.Y);
    }

    public void run(EventManager em) {
        if (deviceNames.isEmpty()) {
            telemetry.addData("No adjustable devices found", "");
            telemetry.update();
            return;
        }

        telemetry.addLine()
                .addData("DPAD", "Device / Setting").setRetained(true)
                .addData("[A]", "Save").setRetained(true)
                .addData("[B]", "Revert").setRetained(true);
        telemetry.addLine()
                .addData("(LS)", "Adjust").setRetained(true)
                .addData("[LB]", "x2").setRetained(true)
                .addData("[RB]", "x5").setRetained(true);
        selectDevice();
        selectSetting();
        updateTelemetry(null);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)!=0) return;
                if (source.isPressed(Button.DPAD_UP)) {
                    if (--deviceIndex < 0) {
                        deviceIndex = deviceNames.size() - 1;
                    }
                } else {
                    if (++deviceIndex >= deviceNames.size()) {
                        deviceIndex = 0;
                    }
                }
                selectDevice();
                if (settingIndex > deviceSettings.size()) settingIndex = 0;
                selectSetting();
                updateTelemetry(null);
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)!=0) return;
                if (source.isPressed(Button.DPAD_LEFT)) {
                    if (--settingIndex < 0) {
                        settingIndex = deviceSettings.size() - 1;
                    }
                } else {
                    if (++settingIndex >= deviceSettings.size()) {
                        settingIndex = 0;
                    }
                }
                selectSetting();
                updateTelemetry(null);
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.START)) return;
                if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)!=0) return;
                if (selectedDevice!=null) selectedDevice.setAdjustmentMode(false);
                if (cfg.storе()) {
                    updateTelemetry("Saved configuration");
                } else {
                    updateTelemetry("Unable to save, see log for details");
                }
            }
        }, Button.A);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.START)) return;
                if (selectedDevice!=null) selectedDevice.setAdjustmentMode(false);
                if (cfg.apply()) {
                    updateTelemetry("Reverted configuration");
                } else {
                    updateTelemetry("Unable to revert, see log for details");
                }
            }
        }, Button.B);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) {
                if (selectedSetting==null || currentY==0) return;
                double step = selectedSetting.getValue().step();
                if (source.isPressed(Button.LEFT_BUMPER)) step *= 2;
                if (source.isPressed(Button.RIGHT_BUMPER)) step *= 5;

                double value = getProperty(selectedDevice, selectedSetting.getKey());
                verbose("stickMove: y=%+.2f, value=%+.3f, step=%+.3f, min=%.2f, max=%.2f",
                        currentY, value, step, selectedSetting.getValue().min(), selectedSetting.getValue().max());
                if (currentY < 0) {
                    value = Math.max(value - step, selectedSetting.getValue().min());
                } else if (currentY > 0) {
                    value = Math.min(value + step, selectedSetting.getValue().max());
                }
                selectedDevice.setAdjustmentMode(true);
                updateProperty(selectedDevice, selectedSetting.getKey(), value);
                verbose("stickMove: newValue=%+.3f", value);
                updateTelemetry(null);
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);
    }

    public double getProperty(Configurable device, String name) {
        try {
            return ((Number) Reflection.get(device, name)).doubleValue();
        } catch (Exception E) {
            error("Unable to get %s: %s", name, E.getMessage(), E);
        }
        return 0;
    }

    public void updateProperty(Configurable device, String name, double value) {
        try {
            Reflection.set(device, name, value);
        } catch (Exception E) {
            error("Unable to set %s = %.2f: %s", name, value, E.getMessage(), E);
        }
    }

    private void selectDevice() {
        selectedDevice = cfg.get(deviceNames.get(deviceIndex));
        deviceSettings = cfg.getAdjustableProperties(selectedDevice);
        verbose("selectDevice: %d %s", deviceIndex, deviceNames.get(deviceIndex));
    }

    private void selectSetting() {
        selectedSetting = null;
        if (deviceSettings.isEmpty()) return;

        int index = 0;
        for (Map.Entry<String, Adjustable> setting : deviceSettings.entrySet()) {
            if (index++ == settingIndex) {
                selectedSetting = setting;
                break;
            }
        } // for
        verbose("selectSetting: %d %s", settingIndex, selectedSetting==null? null : selectedSetting.getKey());
    }

    private void updateTelemetry(String message) {
        String result = "No cfg available";
        if (selectedSetting!=null) {
            result = String.format("%s = %.3f", selectedSetting.getKey(),
                    getProperty(selectedDevice, selectedSetting.getKey())
            );
        }
        telemetry.addData(deviceNames.get(deviceIndex), result);
        if (message!=null) {
            telemetry.addData(message, "");
        }
        telemetry.update();
    }
}
