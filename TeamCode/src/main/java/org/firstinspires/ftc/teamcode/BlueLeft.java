package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Blue_Left", group="Competition")
public class BlueLeft extends OpMode {

    private int stateMachineFlow;
    MecanumDrive robot    = new MecanumDrive();
    Intake intake         = new Intake();
    Shooter shooter       = new Shooter();
    WobbleGrabber grabber = new WobbleGrabber();
    RingCamera    camera  = new RingCamera();

    RingNumber ringNumber = RingNumber.ZERO;

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
                //Lower Wobble Grabber
                grabber.lowerGripper(.5);
                stateMachineFlow++;
                break;
            case 101:
                //Grab wobble goal
                grabber.gripperPosition(1);
                stateMachineFlow++;
                break;
            case 102:
                //Drive forward into zone A
                robot.linearDrive(.5,35);
                stateMachineFlow++;
                break;
            case 103:
                //Release wobble goal
                grabber.gripperPosition(0);
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
                robot.sideDrive(.5,-10);
                stateMachineFlow++;
                break;
            case 107:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 108:
                //Shoot rings into the goal
                stateMachineFlow++;
                break;
            case 109:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 150:
                //Move right in line with first power shot
                robot.sideDrive(.5,-12);
                stateMachineFlow++;
                break;
            case 151:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 152:
                //Shoot first power shot
                stateMachineFlow++;
                break;
            case 153:
                //Move right to second power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 154:
                //Shoot second power shot
                stateMachineFlow++;
                break;
            case 155:
                //Move right to third power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 156:
                //Shoot third power shot
                stateMachineFlow++;
                break;
            case 157:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
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
                //Lower wobble grabber
                grabber.lowerGripper(.5);
                stateMachineFlow++;
                break;
            case 201:
                //Grab wobble goal
                grabber.gripperPosition(1);
                stateMachineFlow++;
                break;
            case 202:
                //Drive forward, to the right, and into zone B
                robot.sideDrive(.5,-10);
                robot.linearDrive(.5,48);
                stateMachineFlow++;
                break;
            case 203:
                //Release wobble goal
                grabber.gripperPosition(0);
                stateMachineFlow++;
                break;
            case 204:
                //Back up behind shot line
                robot.linearDrive(.5,-20);
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
                //Move to be in line with goal
                robot.sideDrive(.5,-1);
                stateMachineFlow++;
                break;
            case 207:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 208:
                //Shoot rings into the goal
                stateMachineFlow++;
                break;
            case 209:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 250:
                //Move right in line with first power shot
                robot.sideDrive(.5,-5);
                stateMachineFlow++;
                break;
            case 251:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 252:
                //Shoot first power shot
                stateMachineFlow++;
                break;
            case 253:
                //Move right to second power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 254:
                //Shoot second power shot
                stateMachineFlow++;
                break;
            case 255:
                //Move right to third power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 256:
                //Shoot third power shot
                stateMachineFlow++;
                break;
            case 257:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
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
                //Lower Wobble Grabber
                grabber.lowerGripper(.5);
                stateMachineFlow++;
                break;
            case 301:
                //Grab wobble goal
                grabber.gripperPosition(1);
                stateMachineFlow++;
                break;
            case 302:
                //Drive forward into zone C
                robot.linearDrive(.5,60);
                stateMachineFlow++;
                break;
            case 303:
                //Release wobble goal
                grabber.gripperPosition(0);
                stateMachineFlow++;
                break;
            case 304:
                //Back up behind shot line
                robot.linearDrive(.5,-24);
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
                robot.sideDrive(.5,-10);
                stateMachineFlow++;
                break;
            case 307:
                //Turn on shooter
                shooter.shooterPower(-.8);
                stateMachineFlow++;
                break;
            case 308:
                //Shoot rings into the goal
                stateMachineFlow++;
                break;
            case 309:
                //Drive forward onto shot line
                robot.linearDrive(.5,5);
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 350:
                //Move right in line with first power shot
                robot.sideDrive(.5,-12);
                stateMachineFlow++;
                break;
            case 351:
                //Turn on the shooter
                shooter.shooterPower(-.75);
                stateMachineFlow++;
                break;
            case 352:
                //Shoot first power shot
                stateMachineFlow++;
                break;
            case 353:
                //Move right to second power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 354:
                //Shoot second power shot
                stateMachineFlow++;
                break;
            case 355:
                //Move right to third power shot
                robot.sideDrive(.5,-2);
                stateMachineFlow++;
                break;
            case 356:
                //Shoot third power shot
                stateMachineFlow++;
                break;
            case 357:
                //Move forward onto the shot line
                robot.linearDrive(.5,5);
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
