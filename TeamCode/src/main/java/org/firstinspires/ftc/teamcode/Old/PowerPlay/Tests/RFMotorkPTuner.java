package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@Autonomous(name = "RFMotorkPTuner")

public class RFMotorkPTuner extends LinearOpMode {
    public void runOpMode() {
        BasicRobot robot = new BasicRobot(this, false);
        double maxTick = 1200, minTick = 0, avg1 = 0, avg2 = 0, loopNums = 0;
        RFMotor rfMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, true, maxTick, minTick);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (rfMotor.getCurrentPosition() < maxTick) {
            rfMotor.setRawPower(0.9);
            avg1 = (avg1 * loopNums + rfMotor.getVelocity()) / (loopNums + 1);
            loopNums++;
            logger.log("/RobotLogs/GeneralRobot", "vel" + rfMotor.getVelocity());

        }
        telemetry.addData("avg1", avg1);
        telemetry.update();
        logger.log("/RobotLogs/GeneralRobot", "avg1" + avg1);
        while (rfMotor.getCurrentPosition() > minTick) {
            rfMotor.setPower(-0.3);
        }
        loopNums = 0;
        sleep(1000);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (rfMotor.getCurrentPosition() < maxTick) {
            rfMotor.setRawPower(1.0);
            logger.log("/RobotLogs/GeneralRobot", "vel" + rfMotor.getVelocity());

            avg2 = (avg2 * loopNums + rfMotor.getVelocity()) / (loopNums + 1);
            loopNums++;
        }
        while (rfMotor.getCurrentPosition() > minTick) {
            rfMotor.setPower(-0.3);
        }
        telemetry.addData("avg1", avg1);
        telemetry.addData("avg2", avg2);
        logger.log("/RobotLogs/GeneralRobot", "avg2" + avg2);
        double kP = 0.1 / (avg2 - avg1);
        telemetry.addData("kP", kP);
        logger.log("/RobotLogs/GeneralRobot", "kP" + kP);
        telemetry.update();
//        double kP = 3.94E-4;
//        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ArrayList<double[]> v1 = new ArrayList<>();
//        while (rfMotor.getCurrentPosition() < maxTick) {
//            rfMotor.setPower(1.0);
//            double position = rfMotor.getCurrentPosition();
//            double velocity = rfMotor.getVelocity();
//            logger.logNoTime("/RobotLogs/GeneralRobot", "" + position + "," + (1.0 / kP - velocity)/velocity+1);
//            double[] data = {position,velocity+1};
//            v1.add(data);
//        }
////        g(x) + v(1.0)*f(x)
//        while (rfMotor.getCurrentPosition() > minTick) {
//            rfMotor.setPower(-0.7);
//            double position = rfMotor.getCurrentPosition();
//            double velocity = rfMotor.getVelocity();
//            logger.logNoTime("/RobotLogs/GeneralRobot", "" + position + "," + (-0.7 / kP - velocity)/velocity+1);
//        }
//        ArrayList<double[]> v2 = new ArrayList<>();
//        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while (rfMotor.getCurrentPosition() < maxTick) {
//            rfMotor.setPower(0.5);
//            double position = rfMotor.getCurrentPosition();
//            double velocity = rfMotor.getVelocity();
//            logger.logNoTime("/RobotLogs/GeneralRobot", "" + position + "," + (0.5 / kP - velocity)/velocity+1);
//            double[] data = {position,velocity+1};
//            v2.add(data);
//        }
//        for(int i=0;i<v1.size();i++){
//            logger.logNoTime("/RobotLogs/GeneralRobot", "" + v1.get(i)[0] + "," + v1.get(i)[1]);
//        }
//        for(int i=0;i<v2.size();i++){
//            logger.logNoTime("/RobotLogs/GeneralRobot", "" + v2.get(i)[0] + "," + v2.get(i)[1]);
//        }
////        g(x) + v(0.5)(x)*f(x) og data
////        g(x)/v(0.5)(x) + f(x) data divide by velocity
////        repeat process for v(1.0)
////        g(x)*(1/v(0.5)(x) - 1/v(1.0)(x)) subtract the two
////        g(x) divide by that thing
//        while (rfMotor.getCurrentPosition() > minTick) {
//            rfMotor.setPower(-0.3);
//        }
    }

}
