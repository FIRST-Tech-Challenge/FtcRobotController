//package org.nknsd.teamcode.components.handlers.launch;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.nknsd.teamcode.components.utility.StateMachine;
//import org.nknsd.teamcode.frameworks.NKNComponent;
//
//public class TrajectoryHandler implements NKNComponent {
//
//    Servo angleServo;
//
//    double targetAngle;
//
//    public void setAngle(double angle){
//        targetAngle = angle;
//        angleServo.setPosition(targetAngle);
//    }
//
//
//    @Override
//    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
//        angleServo = hardwareMap.servo.get("ChuteAdjust");
//        setAngle(1);
//        return true;
//    }
//
//    @Override
//    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
//
//    }
//
//    @Override
//    public void start(ElapsedTime runtime, Telemetry telemetry) {
//    }
//
//    @Override
//    public void stop(ElapsedTime runtime, Telemetry telemetry) {
//    }
//
//    @Override
//    public String getName() {
//        return "";
//    }
//
//    @Override
//    public void loop(ElapsedTime runtime, Telemetry telemetry) {
//    }
//
//    @Override
//    public void doTelemetry(Telemetry telemetry) {
//        telemetry.addData("chute angle", targetAngle);
//    }
//}
