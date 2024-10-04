package org.nknsd.robotics.framework;

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
//    {
        // Construct, link and load
        //      MyComponent myComponent = new MyComponent();
        //      MyComponent2 myComponent2 = new MyComponent2();
        //      myComponent.link(myComponent2);
        //      components.add(myComponent);
        //      components.add(myComponent2);
//    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        createComponents(componentList, enabledTelemetryList);
        for (NKNComponent component:componentList){
            if (!component.init(telemetry,hardwareMap,gamepad1,gamepad2)){
                telemetry.addData("Status", "Failed on "+component.getName());
                telemetry.update();
                throw new NullPointerException("Failed to init "+component.getName());
            }
        }
    }

    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.init_loop(runtime,telemetry);
        }
    }

    public void start(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.start(runtime,telemetry);
        }
    }

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

    public void stop(ElapsedTime runtime, Telemetry telemetry) {
        for (NKNComponent component:componentList){
            component.stop(runtime,telemetry);
        }
    }

    private boolean isInTelemetryEnabledList(NKNComponent component) {
        for (NKNComponent c: enabledTelemetryList) {
            if (c.equals(component)) {
                return true;
            }
        }
        return false;
    }

    public void doTelemetry(Telemetry telemetry) {
        for (NKNComponent component:componentList){
            if (isInTelemetryEnabledList(component)) {
                component.doTelemetry(telemetry);
            }
        }
        telemetry.update();
    }
}
