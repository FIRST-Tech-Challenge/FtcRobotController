//package org.firstinspires.ftc.masters;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import java.util.Date;
//
//@Autonomous(name="Testing CV")
//public class TestingCV extends LinearOpMode {
//
//    TheAbsolutelyPositivelyWithoutAShadowOfADoubtFinalLastIterationOfFreightFrenzyCV CV;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CV = new TheAbsolutelyPositivelyWithoutAShadowOfADoubtFinalLastIterationOfFreightFrenzyCV(hardwareMap, telemetry);
//        TheAbsolutelyPositivelyWithoutAShadowOfADoubtFinalLastIterationOfFreightFrenzyCV.DuckDeterminationPipeline.DuckPosition duckLocation = null;
//
//        waitForStart();
//
//        long startTime = new Date().getTime();
//        long time = 0;
//
//        while (time < 200 && opModeIsActive()) {
//            time = new Date().getTime() - startTime;
//            duckLocation = analyze();
//
//            telemetry.addData("Position", duckLocation);
//            telemetry.update();
//        }
//    }
//    public TheAbsolutelyPositivelyWithoutAShadowOfADoubtFinalLastIterationOfFreightFrenzyCV.DuckDeterminationPipeline.DuckPosition analyze() {
//        return CV.duckPipeline.position;
//    }
//
//}
//
