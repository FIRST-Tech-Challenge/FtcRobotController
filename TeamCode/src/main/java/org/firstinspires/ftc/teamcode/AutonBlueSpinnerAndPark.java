package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name="Blue Right Spinner and Park", group="AB")

public class AutonBlueSpinnerAndPark extends OpMode {
    MecanumDrive robot = new MecanumDrive();
    //    Intake intake = new Intake();
    MotorControl motor = new MotorControl();
    Servos servo = new Servos();

    private int stateMachineFlow;
    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init(){
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 10000;
        robot.initIMU(hardwareMap);
        //Maps all the variables to the correct corresponding  hardware part on the robot
        robot.init(hardwareMap);
//        intake.init(hardwareMap);
        motor.init(hardwareMap);
        servo.init(hardwareMap);

        stateMachineFlow = 0;//used to iterate through the switch loop
    }
    public void loop(){
        switch(stateMachineFlow) {
            case 0:
                waitTime = .2;
                runtime.reset();
                while (waitTime > runtime.time()){
//                    robot.leftBack.setPower(-.2);
//                    robot.leftFront.setPower(-.2);
                }
//                robot.leftBack.setPower(0);
//                robot.leftFront.setPower(0);

                telemetry.addData("GyroAngle", robot.getAngle());
                telemetry.addData("Stage", 1);
                telemetry.update();
                robot.oneSideRotate(-45,.4);
                telemetry.addData("GyroAngle", robot.getAngle());
                telemetry.update();
//                robot.gStatTurn(.2,-6);
//                waitTime = .2;
//                runtime.reset();
//                while (waitTime > runtime.time());
//                robot.frontLinearDrive(.7,-22);
                stateMachineFlow++;
                break;

            case 1:
                telemetry.addData("Stage", 2);
                telemetry.update();
                waitTime = 2.2;
                runtime.reset();
                while (waitTime > runtime.time()) {
                    motor.rotaterPower(.6);
                }
                motor.rotaterPower(0);
                stateMachineFlow++;
                break;
            case 2:
//                waitTime = .5;
//                runtime.reset();
//                while (waitTime > runtime.time());
//                robot.gStatTurn(.4,15);
//
                telemetry.addData("Stage", 3);
                telemetry.update();
                waitTime = .2;
                runtime.reset();
                while (waitTime > runtime.time()){
//                    robot.leftBack.setPower(.2);
//                    robot.leftFront.setPower(.2);
                }
                robot.oneSideRotate(45,.4);

//                robot.leftBack.setPower(0);
//                robot.leftFront.setPower(0);

                stateMachineFlow++;
                break;
            case 3:
                waitTime = .5;
                runtime.reset();
                while (waitTime > runtime.time());
                robot.frontLinearDrive(.8,-30);
                stateMachineFlow++;
                break;
            default:
                stop();
                break;
        }
    }
}
