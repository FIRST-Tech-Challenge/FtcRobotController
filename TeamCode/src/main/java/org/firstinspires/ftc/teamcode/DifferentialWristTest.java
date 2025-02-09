//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.bots.DifferentialWristBot;
//import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
//
//@TeleOp(name = "DifferentialWristTest")
//public class DifferentialWristTest extends LinearOpMode {
//    private DifferentialWristBot differentialWristBot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        differentialWristBot = new DifferentialWristBot(this);
//        differentialWristBot.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            differentialWristBot.onLoop(0,"test");
//            if(gamepad1.dpad_up){
//                rollerIntakeBot.intake();
//            }
//            if(gamepad1.dpad_down){
//                rollerIntakeBot.outake();
//            }
//            if(gamepad1.dpad_left){
//                rollerIntakeBot.stopRoller();
//            }
//            if (gamepad1.a) {
//                rollerIntakeBot.logColorSensor(); // Get colour values
//            }
//
//            telemetry.update();
//        }
//    }
//}