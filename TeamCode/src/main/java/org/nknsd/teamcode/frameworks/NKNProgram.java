package org.nknsd.teamcode.frameworks;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.List;

public abstract class NKNProgram {

    private final List<NKNComponent> componentList = new LinkedList<>();
    private final List<NKNComponent> enabledTelemetryList = new LinkedList<>();



    /**
     * Implement this function with your code here to create your components
     */
    public abstract void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled);

    // During the init of the program
    public void init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        createComponents(componentList, enabledTelemetryList);

        // Report on the success of the component's initialization
        for (NKNComponent component:componentList){
            if (!component.init(telemetry,hardwareMap,gamepad1,gamepad2)){
                telemetry.addData("Status", "Failed on "+component.getName());
                telemetry.update();
                throw new NullPointerException("Failed to init "+component.getName());
            }
        }
    }

    // Code to run REPEATEDLY after hitting INIT but before hitting PLAY
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.init_loop(runtime,telemetry);
        }
    }

    // Code to run ONCE when the driver hits PLAY
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.start(runtime,telemetry);
        }
    }

    // Code to run REPEATEDLY after the driver hits PLAY
    // Does NOT handle telemetry
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            double startTime = runtime.seconds();
            component.loop(runtime, telemetry);
            double duration = runtime.seconds() - startTime;
//            if (duration > .1) {
//                telemetry.addData("HOG", component.getName() + "took "+ duration +"s.");
//            }
        }
    }

    // Code to run ONCE after the driver hits STOP
    public void stop(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.stop(runtime,telemetry);
        }
    }

    // Helper function which checks if the component is telemetry enabled (default: not-enabled)
    boolean isInTelemetryEnabledList(NKNComponent component) {
        for (NKNComponent c: enabledTelemetryList) {
            if (c.equals(component)) {
                return true;
            }
        }
        return false;
    }

    // Runs once every ~500 milliseconds
    // ONLY calls for components to do telemetry
    public void doTelemetry(Telemetry telemetry) {
        for (NKNComponent component:componentList){
            if (isInTelemetryEnabledList(component)) {
                component.doTelemetry(telemetry);
            }
        }
        telemetry.update();
    }
}
