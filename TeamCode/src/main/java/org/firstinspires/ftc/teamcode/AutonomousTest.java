//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.bots.FSMBot;
////import org.firstinspires.ftc.teamcode.bots.HangBot;
//
//@Autonomous(name = "Auto Test", group = "Auto")
//public class AutonomousTest extends LinearOpMode {
//    protected FSMBot robot = new FSMBot(this);
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.isAuto = true;
//        robot.init(hardwareMap);
//
//
//
//        while (!opModeIsActive()) {
//            telemetry.addData("status", "started");
//            telemetry.update();
//            robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//            robot.sleep(500);
//            robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_2;
//        }
//
//        waitForStart();
//        while(opModeIsActive()){
//            robot.onLoop(0,"test");
//
////            robot.updateTelemetry();
////            robot.subIntake(true);
//        }
//
//
//
////        robot.scoreBucket(true);
//    }}