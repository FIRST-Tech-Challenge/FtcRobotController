//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.freightfrenzy.FreightFrenzyRobot;
//
//@TeleOp(name = "FreightFrenzy", group = "TeleOP")
//public class FreightFrenzyTeleOp extends CommandOpMode {
//    private FreightFrenzyRobot robot;
//
//    @Override
//    public void initialize() {
//        robot = new FreightFrenzyRobot(hardwareMap, telemetry, gamepad1, gamepad2);
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        // TODO: Make telemetry subsystem/command and remove this function
//        robot.telemetryUpdate();
//        robot.dashboardTelemetryUpdate();
//    }
//}