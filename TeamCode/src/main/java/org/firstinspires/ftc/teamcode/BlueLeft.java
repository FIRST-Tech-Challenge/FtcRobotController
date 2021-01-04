package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue_Left", group="Competition")
public class BlueLeft extends OpMode {

    private int stateMachineFlow;
    MecanumDrive robot    = new MecanumDrive();
    Intake intake         = new Intake();
    Shooter shooter       = new Shooter();
    WobbleGrabber grabber = new WobbleGrabber();
    RingCamera    camera  = new RingCamera();

    RingNumber ringNumber = RingNumber.ZERO;

    private double waitTime;
    private ElapsedTime runtime = new ElapsedTime();

    //Can be used to choose whether we shoot power shots or in the high goal
    Boolean powerShot = false;

    @Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 10000;

        robot.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        grabber.init(hardwareMap);
        camera.init(hardwareMap);

        stateMachineFlow = 0;
    }
    public void initLoop() {
        if (gamepad2.dpad_up) {
            powerShot = true;
        }if (gamepad2.dpad_down) {
            powerShot = false;
        }
        telemetry.addData("Power Shot", powerShot);
        telemetry.update();
    }

    @Override
    public void loop() {
        switch(stateMachineFlow) {
            case 0:
                //Use the camera to check for number of rings.
                if (camera.ringCount() == 1) {
                    ringNumber = RingNumber.ONE;
                } else if (camera.ringCount() == 4) {
                    ringNumber = RingNumber.FOUR;
                } else {
                    //Zero rings is our default case. In the event that the object recognition fails, we will assume zero.
                    ringNumber = RingNumber.ZERO;
                }
                telemetry.addData("Ring Count",ringNumber);
                telemetry.update();
                stateMachineFlow++;
                break;
            case 1:
                //Use the value found by the camera scan to choose the path. The ZERO case is the default if the camera fails.
                if (ringNumber == RingNumber.ZERO) {
                    stateMachineFlow = 100;
                }else if (ringNumber == RingNumber.ONE) {
                    stateMachineFlow = 200;
                }else if (ringNumber == RingNumber.FOUR) {
                    stateMachineFlow = 300;
                }
                break;

            /***************************
             *
             * Zero Rings on the Field
             *
              **************************/
            case 100:
                //Drive forward into zone A
                robot.linearDrive(.5,50);
                stateMachineFlow++;
                break;
            case 101:
                //Lower Wobble Grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 102:
                //Release wobble goal
                grabber.gripperPosition(0);
                stateMachineFlow++;
                break;
            case 103:
                //Raise wobble grabber
                grabber.gripWrist.setPosition(.75);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 104:
                //Back up behind shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
            case 105:
                /*
                High goal or powershot
                 */
                if (!powerShot) {
                    //High goal
                    stateMachineFlow++;
                } else {
                    //powershot
                    stateMachineFlow = 150;
                }
                break;
            case 106:
                //Move right to be in line with goal
                robot.sideDrive(.5,-20);
                stateMachineFlow++;
                break;
            case 107:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 108:
                //Shoot rings into the goal
                intake.intakePower(-1);
                waitTime = 3;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 109:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 110:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 150:
                //Move right in line with first power shot
                robot.sideDrive(.5,-40);
                stateMachineFlow++;
                break;
            case 151:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 152:
                //Turn on intake and shoot first power shot
                intake.intakePower(-1);
                stateMachineFlow++;
                break;
            case 153:
                //Move right to second power shot
                robot.sideDrive(.5,-4);
                stateMachineFlow++;
                break;
            case 154:
                //Turn off intake
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 155:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 156:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */

            /***************************
             *
             * One Ring on the Field
             *
             **************************/
            case 200:
                //Drive forward, to the right, and into zone B
                robot.linearDrive(.5,5);
                robot.sideDrive(.5,-20);
                robot.linearDrive(.5,71);
                stateMachineFlow++;
                break;
            case 201:
                //Lower wobble grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 202:
                //Release wobble goal
                grabber.gripperPosition(0);
                stateMachineFlow++;
                break;
            case 203:
                //Raise wobber grabber
                grabber.gripWrist.setPosition(.75);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 204:
                //Back up behind shot line
                robot.linearDrive(.5,-31);
                stateMachineFlow++;
                break;
            case 205:
                /*
                High goal or powershot
                 */
                if (!powerShot) {
                    //High goal
                    stateMachineFlow++;
                } else {
                    //powershot
                    stateMachineFlow = 250;
                }
                break;
            case 206:
                //Move to be in line with goal (if needed)
                //robot.sideDrive(.5,-1);
                stateMachineFlow++;
                break;
            case 207:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 208:
                //Shoot rings into the goal
                intake.intakePower(-1);
                waitTime = 3;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 209:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 210:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 250:
                //Move right in line with first power shot
                robot.sideDrive(.5,-16);
                stateMachineFlow++;
                break;
            case 251:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 252:
                //Turn on intake and shoot first power shot
                intake.intakePower(-1);
                stateMachineFlow++;
                break;
            case 253:
                //Move right to second power shot
                robot.sideDrive(.5,-4);
                stateMachineFlow++;
                break;
            case 254:
                //Turn off intake
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 255:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 256:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */

                /***************************
                 *
                 * Four Rings on the Field
                 *
                 **************************/
            case 300:
                //Drive forward into zone C
                robot.linearDrive(.5,100);
                stateMachineFlow++;
                break;
            case 301:
                //Lower Wobble Grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 302:
                //Release wobble goal
                grabber.gripperPosition(0);
                stateMachineFlow++;
                break;
            case 303:
                //Raise wobble grabber
                grabber.gripWrist.setPosition(.75);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 304:
                //Back up behind shot line
                robot.linearDrive(.5,-55);
                stateMachineFlow++;
                break;
            case 305:
                /*
                High goal or powershot
                 */
                if (!powerShot) {
                    //High goal
                    stateMachineFlow++;
                } else {
                    //powershot
                    stateMachineFlow = 350;
                }
                break;
            case 306:
                //Move right to be in line with goal
                robot.sideDrive(.5,-20);
                stateMachineFlow++;
                break;
            case 307:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 308:
                //Shoot rings into the goal
                intake.intakePower(-1);
                waitTime = 3;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 309:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 310:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 350:
                //Move right in line with first power shot
                robot.sideDrive(.5,-40);
                stateMachineFlow++;
                break;
            case 351:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 352:
                //Turn on intake and shoot first power shot
                intake.intakePower(-1);
                stateMachineFlow++;
                break;
            case 353:
                //Move right to second power shot
                robot.sideDrive(.5,-4);
                stateMachineFlow++;
                break;
            case 354:
                //Turn off intake
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 355:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
            case 356:
                //Turn off shooter
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
                /*
                End...
                 */

            default:
                //End program...
                stop();
                break;
        }
    }
}
