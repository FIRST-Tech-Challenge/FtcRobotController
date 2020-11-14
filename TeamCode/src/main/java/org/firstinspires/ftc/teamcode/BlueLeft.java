package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class BlueLeft extends OpMode {

    private int stateMachineFlow;
    MecanumDrive robot    = new MecanumDrive();
    Intake intake         = new Intake();
    Shooter shooter       = new Shooter();
    WobbleGrabber grabber = new WobbleGrabber();

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
                stateMachineFlow++;
                break;
            case 101:
                //Grab wobble grabber
                stateMachineFlow++;
                break;
            case 102:
                //Drive forward to A zone
                stateMachineFlow++;
                break;
            case 103:
                //Back up behind shot line
                stateMachineFlow++;
                break;
            case 104:
                //High goal or powershot
                if (!powerShot) {
                    //High goal
                    stateMachineFlow++;
                } else {
                    //powershot
                    stateMachineFlow = 150;
                }
                break;
            case 105:
                //Move right to be in line with goal
                stateMachineFlow++;
                break;
            case 106:
                //Turn on shooter
                stateMachineFlow++;
                break;
            case 107:
                //Shoot rings into the goal
                stateMachineFlow++;
                break;
            case 108:
                //Drive forward onto shot line
                stateMachineFlow++;
                break;
                /*
                End...
                 */
            case 150:
                //Move right in line with first power shot
                stateMachineFlow++;
                break;
            case 151:
                //Turn on the shooter
                stateMachineFlow++;
                break;
            case 152:
                //Shoot first power shot
                stateMachineFlow++;
                break;
            case 153:
                //Move right to second power shot
                stateMachineFlow++;
                break;
            case 154:
                //Shoot second power shot
                stateMachineFlow++;
                break;
            case 155:
                //Move right to third power shot
                stateMachineFlow++;
                break;
            case 156:
                //Shoot third power shot
                stateMachineFlow++;
                break;
            case 157:
                //Move forward onto the shot line
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
                //Start of the one ring path
                stateMachineFlow++;
                break;

            /***************************
             *
             * Four Rings on the Field
             *
             **************************/
            case 300:
                //Start of the four ring path
                stateMachineFlow++;
                break;

            default:
                //End program...
                stop();
                break;
        }
    }
}
