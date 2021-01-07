package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue_Power_Shot", group="Competition")
public class BluePower extends OpMode{

    /********************
     *
     * Start the robot along the right line
     *
     ********************/

    private int stateMachineFlow;
    MecanumDrive robot    = new MecanumDrive();
    Intake intake         = new Intake();
    Shooter shooter       = new Shooter();
    WobbleGrabber grabber = new WobbleGrabber();

    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 10000;

        robot.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        grabber.init(hardwareMap);

        stateMachineFlow = 0;
    }

    @Override
    public void loop() {
        switch(stateMachineFlow) {
            case 0:
                robot.frontLinearDrive(.45,-85);
                stateMachineFlow++;
                break;
            case 1:
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 2:
                grabber.gripperPosition(.45);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 3:
                grabber.gripWrist.setPosition(.76);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 4:
                robot.frontLinearDrive(.35,22);
                stateMachineFlow++;
                break;
            case 5:
                shooter.shooterPower(-.8);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 6:
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 7:
                robot.sideDrive(.45,-7);
                stateMachineFlow++;
                break;
            case 8:
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
            case 9:
                shooter.shooterPower(0);
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 10:
                robot.frontLinearDrive(.5,-6);
                stateMachineFlow++;
                break;
            case 11:
                intake.lowerIntake();
                stateMachineFlow++;
                break;
            default:
                stop();
                break;

        }
    }
}
