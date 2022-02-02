package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name="Red Left Spinner and Park", group="AB")
public class AutonRedSpinnerAndPark extends OpMode {
    MecanumDrive robot = new MecanumDrive();
    //    Intake intake = new Intake();
    MotorControl motor = new MotorControl();
    Servos servo = new Servos();

    private int stateMachineFlow;
    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    //outside edge lined up with straight line of the carousel
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
                robot.frontLinearDrive(.8,-10);
//                waitTime = .2;
//                runtime.reset();
//                while (waitTime > runtime.time()){}
//                telemetry.addData("GyroAngle", robot.getAngle());
                telemetry.addData("Stage", 1);
                telemetry.update();
//                robot.oneSideRotateReverse(65,-.4);
//                telemetry.addData("GyroAngle", robot.getAngle());
//                telemetry.update();
                stateMachineFlow++;
                break;

            case 1:
//                robot.frontLinearDrive(.8,-15);
                telemetry.addData("Stage", 2);
                telemetry.update();

//                motor.rotaterPower(0);
                robot.gStatTurn(.8,145);
                robot.frontLinearDrive(.3,-12);
                waitTime = 2.8;
                runtime.reset();
                while (waitTime > runtime.time()) {
                    motor.rotaterPower(.85);
                }
                motor.rotaterPower(0);
                stateMachineFlow++;
                break;
            case 2:

                telemetry.addData("Stage", 3);
                telemetry.update();
                waitTime = .2;
                runtime.reset();
                while (waitTime > runtime.time()){
//                    robot.leftBack.setPower(.2);
//                    robot.leftFront.setPower(.2);
                }
                robot.frontLinearDrive(.8,13);
                robot.gStatTurn(.8,-56);

//                robot.oneSideRotate(-65,.4);

//                robot.leftBack.setPower(0);
//                robot.leftFront.setPower(0);

                stateMachineFlow++;
                break;
            case 3:
                waitTime = .5;
                runtime.reset();
                while (waitTime > runtime.time());
                robot.frontLinearDrive(.4,-20);

//                robot.frontLinearDrive(.8,-25);
                stateMachineFlow++;
                break;
            default:
                stop();
                break;
        }
    }
}
