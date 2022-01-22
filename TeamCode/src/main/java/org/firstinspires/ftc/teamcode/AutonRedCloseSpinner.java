package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Red Spinner Close", group="AR")
public class AutonRedCloseSpinner extends OpMode{
    MecanumDrive robot = new MecanumDrive();
    //    Intake intake = new Intake();
    MotorControl motor = new MotorControl();
    Servos servo = new Servos();

    private int stateMachineFlow;
    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();

    public void init(){
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 10000;

        //Maps all the variables to the correct corresponding  hardware part on the robot
        robot.init(hardwareMap);
//        intake.init(hardwareMap);
        motor.init(hardwareMap);
        servo.init(hardwareMap);

        stateMachineFlow = 0;
    }
    public void loop(){
        switch(stateMachineFlow) {
            case 0:
                waitTime = .5;
                runtime.reset();
                while (waitTime > runtime.time());
                robot.gStatTurn(.8,-90);
                stateMachineFlow++;
                break;

            case 1:
                waitTime = 5;
                runtime.reset();
                while (waitTime > runtime.time());
//
                break;
            case 2:
//
                robot.frontLinearDrive(.8,22);
                stateMachineFlow++;
                break;
            case 3:
                waitTime = .8;
                runtime.reset();
                while(waitTime > runtime.time()){
                    motor.rotaterPower(.6);
                }
                motor.rotaterPower(0);
                stateMachineFlow++;
                break;
            default:
                stop();
        }
    }
}
