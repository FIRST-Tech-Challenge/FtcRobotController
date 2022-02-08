package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name="Complete Red Auton", group="AB")

public class TotalRedAuton extends OpMode{
    MecanumDrive robot = new MecanumDrive();
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
        motor.init(hardwareMap);
        servo.init(hardwareMap);
        stateMachineFlow = 0;//used to iterate through the switch loop
    }
    public void loop(){
        switch(stateMachineFlow){
            case 0:
                servo.changePos(0.18);
                robot.frontLinearDrive(.5,18.5);
                telemetry.addData("Stage", 1);
                telemetry.update();
                waitTime = .4;
                runtime.reset();
                while (waitTime > runtime.time());
                motor.verLiftPos(.9,80);
                stateMachineFlow++;
                break;
            case 1:
                waitTime = 1;
                runtime.reset();
                while (waitTime > runtime.time()){
                    motor.HorzLift.setPower(-.6);
                }
                motor.HorzLift.setPower(0);
//                motor.verLiftPos(.6,80);
                servo.changePos(.8);
                waitTime = .4;
                runtime.reset();
                while (waitTime > runtime.time()){
                }
                servo.changePos(0.18);
                stateMachineFlow++;
                break;
            case 2:
                waitTime = 1.05;
                runtime.reset();
                while (waitTime > runtime.time()){
                    motor.HorzLift.setPower(.6);
                }
                motor.HorzLift.setPower(0);
                robot.frontLinearDrive(.4,-3);
                motor.verLiftPos(.9,-50);
                stateMachineFlow++;
                break;
            case 3:
                robot.gStatTurn(.65,-50);
                runtime.reset();
                while (waitTime > runtime.time());
                stateMachineFlow++;
                break;
            case 4:
                robot.frontLinearDrive(.5,-52);
                robot.gStatTurn(.3,38);
                robot.frontLinearDrive(.1,-.5);

//                robot.frontLinearDrive(.1,-2);
                motor.rotaterPower(.85);
                stateMachineFlow++;
                break;
            case 5:
                runtime.reset();
                waitTime = 2.8;
                while (waitTime > runtime.time());
                motor.rotaterPower(0);
                stateMachineFlow++;
                break;
            case 6:
                robot.frontLinearDrive(.65,16);
                robot.gStatTurn(.4,-60);
                robot.frontLinearDrive(1,-10);
//                robot.frontLinearDrive(1,-20);
//                robot.sideDrive(.4,20);
//                robot.frontLinearDrive(1,-110);
                motor.verLiftPos(1,-30);
                stateMachineFlow++;
                break;
            default:
                stop();
                break;
        }
    }
}
