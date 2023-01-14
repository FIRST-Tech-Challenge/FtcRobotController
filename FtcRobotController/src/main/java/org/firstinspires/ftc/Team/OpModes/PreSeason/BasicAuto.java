package org.firstinspires.ftc.Team.OpModes.PreSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team.ComplexRobots.Robot;
import org.firstinspires.ftc.Team.Enums.Direction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Basic Test Autonomous")
public class BasicAuto extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        //Add Built in functions here
    }

    private void drive(double pow, double time) {
        double t = time*1000;
        int t1 = (int)t;
        robot.motors[robot.RFMotor].setPower(pow);
        robot.motors[robot.RBMotor].setPower(pow);
        robot.motors[robot.LFMotor].setPower(pow);
        robot.motors[robot.LBMotor].setPower(pow);
        sleep(t1);
        stopMotors();
    }

    private void stopMotors(){
        robot.stop();
    }

//    public void strafe2(Direction direction, double power, double t) {
//        if(direction == Direction.LEFT) {
//            robot.LFMotor.setPower(-power);
//            robot.RFMotor.setPower(power);
//            robot.LBMotor.setPower(power);
//            robot.RBMotor.setPower(-power);
//        } else if(direction == Direction.RIGHT) {
//            robot.LFMotor.setPower(power);
//            robot.RFMotor.setPower(-power);
//            robot.LBMotor.setPower(-power);
//            robot.RBMotor.setPower(power);
//        }
//        sleep((int)t*1000);
//        stopMotors();
//    }
}