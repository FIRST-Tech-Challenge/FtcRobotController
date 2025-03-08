//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
//
//@TeleOp(name = "IntakeTest2")
//public class IntakeTest2 extends  LinearOpMode{
//    private RollerIntakeBot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new RollerIntakeBot(this);
//        robot.init(hardwareMap);
//
//        waitForStart();
//        while(opModeIsActive()){
//            robot.onLoop(0,"test");
//
//                robot.intake(gamepad1.a);
//                robot.outake(gamepad1.b);
//            if(gamepad1.dpad_up){
//                robot.pitch(0.7);
//            }
//            if(gamepad1.dpad_down){
//                robot.pitch(-0.7);
//            }
//            if(gamepad1.dpad_left){
//                robot.roll(0.7);
//            }
//            if(gamepad1.dpad_right){
//                robot.roll(-0.7);
//            }
//            telemetry.addData("right Target" ,robot.getRightPos());
//            telemetry.addData("left Target" ,robot.getLeftPos());
//            telemetry.addData("Current roll", robot.getCurrentPitch());
//            telemetry.addData("Current pitch", robot.getCurrentRoll());
//            telemetry.update();
//
//        }
//
//    }
//
//}
