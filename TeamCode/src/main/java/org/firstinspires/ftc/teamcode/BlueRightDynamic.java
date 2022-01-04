package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="Blue_Right_Dynamic", group="Competition")
public class BlueRightDynamic extends OpMode {

    /****************************
     *
     * Start the robot to the right of the blue start lines
     *
     ****************************/
    private int stateMachineFlow;
    MecanumDrive robot    = new MecanumDrive();
    Intake intake         = new Intake();
    Shooter shooter       = new Shooter();
    WobbleGrabber grabber = new WobbleGrabber();
    RingCamera    camera  = new RingCamera();

    RingNumber ringNumber = RingNumber.ZERO;

    //Can be used to choose whether we shoot power shots or in the high goal
    Boolean powerShot = true;

    //Choose shooter power-level
    boolean highVoltage = true;
    double power = -.825;
    double high  = -.9;

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
        camera.init(hardwareMap);
        robot.initIMU(hardwareMap);

        if (shooter.scalePowerShot() != Double.POSITIVE_INFINITY) {
            //power = shooter.scalePowerShot();
            power = shooter.scalePowerShotDynamic();
            //high  = shooter.scaleHighGoal();
            high = shooter.scaleHighGoalDynamic();
        }

        stateMachineFlow = 0;
    }
    public void init_loop() {
        //Choose whether or not we shoot power shots
        //input the relative change of the battery to determine the power of the shooter
        if (gamepad2.dpad_up) {
            powerShot = true;
        }if (gamepad2.dpad_down) {
            powerShot = false;
        }

        //This is a fail-safe in case the automatic adjuster doesn't work
        if (gamepad2.dpad_right) {
            power = -.825;
            high = -.9;
            highVoltage = true;
        }if (gamepad2.dpad_left) {
            power = -.875;
            high = -.93;
            highVoltage = false;
        }

        telemetry.addData("Volts", shooter.getBatteryVoltage());
        telemetry.addData("High Goal Power", high);
        telemetry.addData("Power Shot Power", power);
        telemetry.addData("Power Shot", powerShot);
        telemetry.update();
    }

    public void shoot() {
        shooter.shooterSwitch.setPosition(.63);
        waitTime = .3;
        runtime.reset();
        time = runtime.time();
        while (waitTime > runtime.time() - time) {

        }
        shooter.shooterSwitch.setPosition(1);
        waitTime = .3;
        runtime.reset();
        time = runtime.time();
        while (waitTime > runtime.time() - time) {

        }
    }

    @Override
    public void loop() {

        if (shooter.scalePowerShot() != Double.POSITIVE_INFINITY) {
            //power = shooter.scalePowerShot();
            power = shooter.scalePowerShotDynamic();
            //high  = shooter.scaleHighGoal();
            high = shooter.scaleHighGoalDynamic();
        }
        telemetry.addData("Volts", shooter.getBatteryVoltage());
        telemetry.addData("High Goal Power", high);
        telemetry.addData("Power Shot Power", power);
        telemetry.update();

        switch(stateMachineFlow) {
            case 0:
                robot.resetAngle();
                //Drive into place to check number of rings
                robot.linearDrive(.45,-34);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
               // intake.lowerIntake();
                stateMachineFlow++;
                break;
            case 1:
                //Use the camera to check for number of rings.
                if (camera.ringCount() == 1) {
                    ringNumber = RingNumber.ONE;
                } else if (camera.ringCount() == 4) {
                    ringNumber = RingNumber.FOUR;
                } else {
                    //Zero rings is our default case. In the event that the object recognition fails, we will assume zero.
                    ringNumber = RingNumber.ZERO;
                }
                stateMachineFlow++;
                break;
            case 2:
                //Use the value found by the camera scan to choose the path. The ZERO case is the default if the camera fails.
                if (ringNumber == RingNumber.ZERO) {
                    stateMachineFlow = 100;
                }else if (ringNumber == RingNumber.ONE) {
                    stateMachineFlow = 200;
                }else if (ringNumber == RingNumber.FOUR) {
                    stateMachineFlow = 300;
                }
                camera.decativate();
                telemetry.addData("Case",stateMachineFlow);
                break;

            /***************************
             *
             * Zero Rings on the Field
             *
              **************************/
            case 100:
                //Drive forward
                robot.linearDrive(.5,-42);
                stateMachineFlow++;
                break;
            case 101:
                //Drive left into zone A
                robot.sideDrive(.4,28);
                stateMachineFlow++;
                break;
            case 102:
                //Lower Wobble Grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 103:
                //Release wobble goal
                grabber.gripperPosition(0);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 104:
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;

            case 105:
                //Back up behind shot line
                robot.gStatTurn(.2, -robot.getAngle());
                robot.linearDrive(.5,13);
                stateMachineFlow++;
                break;
            case 106:
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
                telemetry.addData("Case",stateMachineFlow);
                break;
            case 107:
                //Move right to be in line with goal
                robot.sideDrive(.4, -4);
                stateMachineFlow++;
                break;
            case 108:
                //Turn on shooter
                shooter.shooterPower(high);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(high);
                }
                stateMachineFlow++;
                break;
            case 109:
                //Shoot rings into the goal
                shoot();
                shoot();
                shoot();
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 110:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 150:
                //Move right in line with first power shot
                robot.sideDrive(.4,-24);
              //  robot.gStatTurn(.2, -robot.getAngle());
                stateMachineFlow++;
                break;
            case 151:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 152:
                //Shoot first power shot
                shoot();
                stateMachineFlow++;
                break;
            case 153:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 154:
                //Shoot second power shot
                shoot();
                stateMachineFlow++;
                break;
            case 155:
                //Move right to third power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 156:
                //shoot third power shot
                shoot();
                stateMachineFlow++;
                break;
            case 157:
                //Turn off shooter and intake
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 158:
                //Move forward onto the shot line
                robot.linearDrive(.5,-7);
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
                //Drive forward to zone B
                robot.linearDrive(.5,-60);
                stateMachineFlow++;
                break;
            case 201:
                //move into zone B
                robot.sideDrive(.4,9);
                stateMachineFlow++;
                break;
            case 202:
                stateMachineFlow++;
                break;
            case 203:
                //Lower wobble grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 204:
                //Release wobble goal
                grabber.gripperPosition(0);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 205:
                //Raise grabber
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 206:

            //Back up behind shot line
            robot.gStatTurn(.2, -robot.getAngle());
            robot.linearDrive(.5,31);
            stateMachineFlow++;
            break;
            case 207:
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
                telemetry.addData("Case",stateMachineFlow);
                break;
            case 208:
                //Move to be in line with goal
                robot.sideDrive(.4,13);
                stateMachineFlow++;
                break;
            case 209:
                //Turn on shooter
                shooter.shooterPower(high);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(high);
                }
                stateMachineFlow++;
                break;
            case 210:
                //Shoot rings into the goal
                shoot();
                shoot();
                shoot();
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 211:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 250:
                //Move right in line with first power shot
                robot.sideDrive(.4,-7);
                stateMachineFlow++;
                break;
            case 251:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 252:
                //Shoot first power shot
                shoot();
                stateMachineFlow++;
                break;
            case 253:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 254:
                //Shoot second power shot
                shoot();
                stateMachineFlow++;
                break;
            case 255:
                //Move right to third power shot
                robot.sideDrive(.4,-7);
            //    robot.gStatTurn(.2, -robot.getAngle());
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 256:
                //shoot third power shot
                shoot();
                stateMachineFlow++;
                break;
            case 257:
                //Turn off shooter and intake
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 258:
                //Move forward onto the shot line
                robot.linearDrive(.5,-7);
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
                //Drive forward
                robot.linearDrive(.5,-84);
                stateMachineFlow++;
                break;
            case 301:
                //Drive to zone C
                robot.sideDrive(.4,30);
                stateMachineFlow++;
                break;
            case 302:
                //Lower Wobble Grabber
                grabber.lowerGripper();
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 303:
                //Release wobble goal
                grabber.gripperPosition(0);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 304:
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 305:
            //Back up behind shot line
            robot.gStatTurn(.2, -robot.getAngle());
            robot.linearDrive(.5,54);
            stateMachineFlow++;
            break;
            case 306:
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
                telemetry.addData("Case",stateMachineFlow);
                break;
            case 307:
                //Move right to be in line with goal
                robot.sideDrive(.4,-6);
                stateMachineFlow++;
                break;
            case 308:
                //Turn on shooter
                shooter.shooterPower(high);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(high);
                }
                stateMachineFlow++;
                break;
            case 309:
                //Shoot rings into the goal
                shoot();
                shoot();
                shoot();
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 310:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 350:
                //Move right in line with first power shot
                robot.sideDrive(.4,-24);
               // robot.gStatTurn(.2, -robot.getAngle());
                stateMachineFlow++;
                break;
            case 351:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 352:
                //Shoot first power shot
                shoot();
                stateMachineFlow++;
                break;
            case 353:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 354:
                //Shoot second power shot
                shoot();
                stateMachineFlow++;
                break;
            case 355:
                //Move right to third power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {
                    shooter.shooterPower(power);
                }
                stateMachineFlow++;
                break;
            case 356:
                //shoot third power shot
                shoot();
                stateMachineFlow++;
                break;
            case 357:
                //Turn off shooter and intake
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 358:
                //Move forward onto the shot line
                robot.linearDrive(.5,-7);
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
