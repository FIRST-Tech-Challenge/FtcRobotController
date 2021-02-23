package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue_Right", group="Competition")
public class BlueRight extends OpMode {

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

        if (shooter.scalePowerShot() != Double.POSITIVE_INFINITY) {
            power = shooter.scalePowerShot();
            high  = shooter.scaleHighGoal();
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

    @Override
    public void loop() {

        if (shooter.scalePowerShot() != Double.POSITIVE_INFINITY) {
            power = shooter.scalePowerShot();
            high  = shooter.scaleHighGoal();
        }
        telemetry.addData("Volts", shooter.getBatteryVoltage());
        telemetry.addData("High Goal Power", high);
        telemetry.addData("Power Shot Power", power);
        telemetry.update();

        switch(stateMachineFlow) {
            case 0:
                //Drive into place to check number of rings
                robot.linearDrive(.45,-34);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.lowerIntake();
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
                //Back up behind shot line
                robot.linearDrive(.5,17);
                stateMachineFlow++;
                break;
            case 105:
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
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

                }
                stateMachineFlow++;
                break;
            case 109:
                //Shoot rings into the goal
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 110:
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 111:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 150:
                //Move right in line with first power shot
                robot.sideDrive(.4,-31);
                stateMachineFlow++;
                break;
            case 151:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 152:
                //Shoot first power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 153:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 154:
                //Shoot second power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 155:
                //Turn off shooter and intake
                shooter.shooterPower(0);
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 156:
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
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 203:
                //Back up behind shot line
                robot.linearDrive(.5,35);
                stateMachineFlow++;
                break;
            case 204:
                //Raise grabber
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
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
                telemetry.addData("Case",stateMachineFlow);
                break;
            case 206:
                //Move to be in line with goal
                robot.sideDrive(.4,22);
                stateMachineFlow++;
                break;
            case 207:
                //Turn on shooter
                shooter.shooterPower(high);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 208:
                //Shoot rings into the goal
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 209:
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 210:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 250:
                //Move right in line with first power shot
                robot.sideDrive(.4,-2);
                stateMachineFlow++;
                break;
            case 251:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 252:
                //Shoot first power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 253:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 254:
                //Shoot second power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 255:
                //Turn off shooter and intake
                intake.intakePower(0);
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 256:
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
                //Back up behind shot line
                robot.linearDrive(.5,57);
                stateMachineFlow++;
                break;
            case 305:
                grabber.gripWrist.setPosition(.23);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
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

                }
                stateMachineFlow++;
                break;
            case 309:
                //Shoot rings into the goal
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 310:
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 311:
                //Drive forward onto shot line
                robot.linearDrive(.5,-7);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 350:
                //Move right in line with first power shot
                robot.sideDrive(.4,-29);
                stateMachineFlow++;
                break;
            case 351:
                //Turn on the shooter
                shooter.shooterPower(power);
                waitTime = 1;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 352:
                //Shoot first power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                intake.intakePower(0);
                stateMachineFlow++;
                break;
            case 353:
                //Move right to second power shot
                robot.sideDrive(.4,-7);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 354:
                //Shoot second power shot
                intake.intakePower(1);
                waitTime = .5;
                runtime.reset();
                time = runtime.time();
                while (waitTime > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 355:
                //Turn off shooter and intake
                intake.intakePower(0);
                shooter.shooterPower(0);
                stateMachineFlow++;
                break;
            case 356:
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
