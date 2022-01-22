package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//@Disabled
@Autonomous(name="Blue Left Park", group="AB")
public class AutonBlueCloseSpinner extends OpMode {
    MecanumDrive robot = new MecanumDrive();
//    Intake intake = new Intake();
    MotorControl motor = new MotorControl();
    Servos servo = new Servos();

    private int stateMachineFlow;
    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    // Does this on initial start up of program
    public void init() {
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

    // Does this after start up, actual code and movement of BOT
    @Override
    public void loop() {
        switch(stateMachineFlow) {
            case 0:
                waitTime = .2;
                runtime.reset();
                while (waitTime > runtime.time());
                robot.frontLinearDrive(.7,-22);
//                telemetry.addData("Intake Power", "I am in case 0.");
//                telemetry.update();
//                waitTime = 3;
//                runtime.reset();
//                time = runtime.seconds();
//                robot.leftBack.setPower(10);
//                robot.leftFront.setPower(10);
//                robot.rightBack.setPower(10);
//                robot.rightFront.setPower(10);
//                waitTime = .5;
//                runtime.reset();
//                while (waitTime > runtime.time());
//                robot.leftBack.setPower(-10);
//                robot.leftFront.setPower(10);
//                robot.rightBack.setPower(10);
//                robot.rightFront.setPower(-10);
//                telemetry.addData("Intake Power", "I am at the end of case 0.");
//                telemetry.update();
                stateMachineFlow++;
                break;

            case 1:
                waitTime = .5;
                runtime.reset();
                while (waitTime > runtime.time());
                robot.gStatTurn(.8,84);
//                waitTime = 5;
//                runtime.reset();
//                while (waitTime > runtime.time());
//                waitTime = .5;
//                runtime.reset();
//                while (waitTime > runtime.time());
//                telemetry.addData("Intake Power", "I am in case 1 at the top");
//                telemetry.update();
//
//                robot.leftBack.setPower(0);
//                robot.leftFront.setPower(0);
//                robot.rightBack.setPower(0);
//                robot.rightFront.setPower(0);
//                telemetry.addData("Intake Power", "I am at the end of case 1.");
//                telemetry.update();
                stateMachineFlow++;
                break;
            case 2:
//                grabber.gripperPosition(0);
//                waitTime = .5;
//                runtime.reset();
//                time = runtime.time();
//                while (waitTime > runtime.time() - time) {
//
//                }
                robot.frontLinearDrive(100,-72);
                stateMachineFlow++;
                break;
            case 3:
//                waitTime = .8;
//                runtime.reset();
//                while(waitTime > runtime.time()){
//                    motor.rotaterPower(.6);
//                }
//                motor.rotaterPower(0);
////                grabber.gripWrist.setPosition(.23);
////                waitTime = 1;
////                runtime.reset();
////                time = runtime.time();
////                while (waitTime > runtime.time() - time) {
////
////                }
                stateMachineFlow++;
                break;
//            case 4:
//                robot.frontLinearDrive(.35,22);
//                stateMachineFlow++;
//                break;
//            case 5:
//                shooter.shooterPower(-.825);
//                waitTime = 1;
//                runtime.reset();
//                time = runtime.time();
//                while (waitTime > runtime.time() - time) {
//
//                }
//                stateMachineFlow++;
//                break;
//            case 6:
//                intake.intakePower(1);
//                waitTime = .5;
//                runtime.reset();
//                time = runtime.time();
//                while (waitTime > runtime.time() - time) {
//
//                }
//                intake.intakePower(0);
//                stateMachineFlow++;
//                break;
//            case 7:
//                robot.sideDrive(.45,-8);
//                waitTime = .5;
//                runtime.reset();
//                time = runtime.time();
//                while (waitTime > runtime.time() - time) {
//
//                }
//                stateMachineFlow++;
//                break;
//            case 8:
//                intake.intakePower(1);
//                waitTime = .5;
//                runtime.reset();
//                time = runtime.time();
//                while (waitTime > runtime.time() - time) {
//
//                }
//            case 9:
//                shooter.shooterPower(0);
//                intake.intakePower(0);
//                stateMachineFlow++;
//                break;
//            case 10:
//                robot.frontLinearDrive(.5,-6);
//                stateMachineFlow++;
//                break;
//            case 11:
//                intake.lowerIntake();
//                stateMachineFlow++;
//                break;
            default:
                stop();
                break;

        }
    }
}
