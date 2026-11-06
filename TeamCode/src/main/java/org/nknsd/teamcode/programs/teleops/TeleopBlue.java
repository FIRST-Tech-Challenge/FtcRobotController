//package org.nknsd.teamcode.programs.teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.nknsd.teamcode.components.handlers.vision.ID;
//import org.nknsd.teamcode.components.utility.RobotVersion;
//import org.nknsd.teamcode.frameworks.NKNComponent;
//import org.nknsd.teamcode.frameworks.NKNProgram;
//import org.nknsd.teamcode.programs.parts.GamepadPart;
//import org.nknsd.teamcode.programs.parts.Setup;
//
//import java.util.List;
//
//@TeleOp(name = "teleop blue")
//public class TeleopBlue extends NKNProgram {
//    @Override
//    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
//        RobotVersion.setRobotAlliance(ID.BLUE);
//        RobotVersion.setIsAutonomous(false);
//
//        Setup setup = new Setup();
//        setup.changeEnableSettings(true,true);
//        setup.createComponents(components,telemetryEnabled);
//
////        telemetryEnabled.add(setup.getBalancedLiftHandler());
////        telemetryEnabled.add(setup.getSrsHubHandler());
//
//        GamepadPart gamepadPart = new GamepadPart(setup);
//        gamepadPart.createComponents(components,telemetryEnabled);
//    }
//}