package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_ALLIANCE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_SHARED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_TRANSFER;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.SEQUENCING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretAAStates.TURRET_RAISED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_STRAIGHT;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFTurret;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.CVPipelines.OpenCVMasterclass;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.BasicChassis;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.CarouselCR;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.ChassisFactory;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.LedColor;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.Turret;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.tseDepositor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

public class BlackoutRobot extends BasicRobot {

    public final static boolean isCorgi = true;
    boolean shouldIntake = true;
    public static int isBlue = 1;
    boolean flipped = false;
    double slowTime = 0.0;
    public static boolean isBall = false;
    public static boolean resetten = true;
    public static boolean faked = false;
    boolean outModed = false;
    double trueStartAngle = 0;
    boolean shouldFlipIntake = false;
    boolean isReversing = false;
    double shareFlipTime = 0, startRotateTime;
    boolean isFlipping = false;
    double flipDelay = .3, reverseDelay = .7;
    double[] startTime = {-2, 0, 0, 0, 0, 0, 0, -10, 0, 100, 0, 0, 0};
    double magnitude;
    double angleInRadian;
    double angleInDegree;
    public static double startAngle;
    double power = 0.5;
    double turret_angle_control_distance = 0;
    boolean slowMode = false;
    boolean autoAiming = false;
    boolean shared_shipping_hub = false;
    boolean alliance_shipping_hub = false;
    public static boolean retracting = false;
    double time;
    boolean changed = false;
    boolean ultra = true;
    boolean touch = false;
    public static double loopTime = 0;
    double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 30;     // period of each cycle


    double MAX_POS = 1.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position

    boolean rampUp = true;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    int x = 0;

    // Hardware Objects
    private BasicChassis drivetrain = null;
    private CarouselCR rotation = null;
    private Intake intake = null;
    private Turret turret = null;
    private RFTurret rfTurret = null;
    private LedColor led_bank = null;
    private OpenCVMasterclass openCV = null;
    private tseDepositor TSE = null;
    public static StateMachine checker = null;

    public BlackoutRobot(LinearOpMode opMode, BasicChassis.ChassisType chassisType, boolean isTeleop, boolean vuforiaNAVIGATIONneeded, double startAng) {
        super(opMode, isTeleop);

        ArrayList<Double> rotationCoefs = new ArrayList<>();
        rotationCoefs.add(4.0);
        rotationCoefs.add(400.0);

        //        startAngle = startAng;
//        trueStartAngle=startAng;
        checker = new StateMachine(isTeleop);
        //This link has a easy to understand explanation of ClassFactories. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
        drivetrain = ChassisFactory.getChassis(chassisType,vuforiaNAVIGATIONneeded, isTeleop);
        rotation = new CarouselCR();
        intake = new Intake( isTeleop, checker);
//        led_bank = new LedColor(op); //LED has to be declared before calling it
        turret = new Turret(led_bank, isTeleop, checker);
        openCV = new OpenCVMasterclass();
        TSE = new tseDepositor(isTeleop);
        rfTurret = new RFTurret("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,
                true, rotationCoefs, 570, -570);
//        checker = new StateMachine(op, isTeleop, logger);
//        //This link has a easy to understand explanation of ClassFactories. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
//        drivetrain = ChassisFactory.getChassis(chassisType, op, vuforiaNAVIGATIONneeded, isTeleop, logger);
//        rotation = new CarouselCR(op);
//        intake = new Intake(op, isTeleop, checker);
////        led_bank = new LedColor(op); //LED has to be declared before calling it
//        turret = new Turret(op, led_bank, isTeleop, checker);
//        openCV = new OpenCVMasterclass(op);
//        TSE = new tseDepositor(op, isTeleop);
//        ultras = new Ultrasonics();
//        touchs = new LimitSwitches();
//        imu = new IMU();
//        roadrun = new SampleMecanumDrive(op.hardwareMap, this);

    }


    public void setFirstLoop(boolean value) {
        queuer.setFirstLoop(value);
    }

    public int BlueElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.BlueTeamElem();
    }
    public int RedElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.RedTeamElem();
    }

    public void flipBasketArmToPosition(double position) {
        turret.FlipBasketArmToPosition(position);
    }

    public void flipBasketToPosition(double position) {
        turret.FlipBasketToPosition(position);
    }

    public void flipBasket() {
        turret.FlipBasket(0);
    }

    public void flipBasketArmHigh() {
        turret.FlipBasketArmHigh();
    }

    public double[] BlueWarehouseScam() {
        return openCV.BlueWarehouseScam();
    }

    public void stopAllMotors() {
        drivetrain.stopAllMotors();
    }

    public void rotateTo (double position) {
        if (queuer.queue(true, abs(position- rfTurret.getCurrentPosition())<10)) {
            rfTurret.setPosition(position);
        }
    }


    /*/******** Left Front Motor **********/
   /* public void moveMotorLeftFront(double distance) {
        drivetrain.moveMotorLeftFront(distance);
    }

    /******** Right Front Motor **********/
   /* public void moveMotorRightFront(double distance) {
        drivetrain.moveMotorRightFront(distance);
    }

    /******** Left Back Motor **********/
    /*public void moveMotorLeftBack(double distance) {
        drivetrain.moveMotorLeftBack(distance);
    }

    /******** Right Back Motor **********/
    /*public void moveMotorRightBack(double distance) {
        drivetrain.moveMotorRightBack(distance);
    }*/

    /******** shooterMotor **********/
    /*public void moveShooterMotor(int distance, int power) {
        shooter.moveShooterMotor(distance, power);
    }
    public double getAngle() {
        return drivetrain.getAngle();
    }


    /**
     * Directional Movement
     **/
    public void moveMultidirectional(double power, double angle, float rightStick, boolean isSlow) {
        drivetrain.moveMultidirectional(power, angle, rightStick, isSlow);
    }

    public void moveMultidirectionalMecanum(double power, float leftStickx, float leftSticky, float rightStick) {
        drivetrain.moveMultidirectionalMecanum(power, leftStickx, leftSticky, rightStick);
    }

    public void goToPositionWithoutStop(int direction, double yPosition, double xPosition, double power) {
        drivetrain.goToPositionWithoutStop(direction, yPosition, xPosition, power);
    }

    public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power, double targetAnglu) {
        drivetrain.tripleSplineToPosition(direction, x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, power, targetAnglu);
    }

    public void tripleSplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power) {
        drivetrain.tripleSplineToPositionHead(direction, x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, power);
    }

    public void partOfPolySplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power, double targetAnglu) {
        drivetrain.partOfPolySplineToPosition(direction, x0, y0, x1, y1, x2, y2, x3, y3, start, end, targetAnglu, power);
    }

    public void partOfPolySplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power) {
        drivetrain.partOfPolySplineToPositionHead(direction, x0, y0, x1, y1, x2, y2, x3, y3, start, end, power);
    }
    /** in the future, we will move this into a different class, robot class needs to be more tidy, with queue
     * system robot classes will be more complex, get rid of empty space**/
    public void teleopLoop(int red, double startx, double starty) {
        /** gamepad 1**/
        isBlue = red;
        startAngle = trueStartAngle * red;
        float forward = -op.gamepad1.left_stick_y;
        float strafe = -op.gamepad1.left_stick_x; //remove dis boi son// DIY!
        float turning = -op.gamepad1.right_stick_x;
        boolean startIntake = op.gamepad1.right_bumper;
        boolean reverseIntake = op.gamepad1.y;
        boolean flipIntake = op.gamepad1.left_bumper;
//        boolean extendTSE = op.gamepad2.dpad_right;
//        boolean manualretractTSE = op.gamepad2.dpad_left;
//        boolean autoretractTSE = op.gamepad1.x;
//        boolean sequence = op.gamepad1.dpad_up;
        boolean TSEArmUp = op.gamepad1.dpad_up;
        /** gamepad 2**/
        float turretTurn = op.gamepad2.left_stick_x;
        float turretUpNDown = op.gamepad2.right_stick_y;
        float extendTurret = -op.gamepad2.left_stick_y;
//        boolean extendAutoTSE = op.gamepad2.dpad_up;
        boolean autoretractTurret = op.gamepad2.y;
        boolean basketArm = op.gamepad2.right_bumper;
        boolean autoAim = op.gamepad2.left_bumper;
        boolean basket = op.gamepad2.a;
        boolean resetTurret = false;
        boolean plusTurret = false;
        boolean slow = op.gamepad1.x;
        boolean unsave_turret_position = false;
        boolean capper = false;

        if (slow && op.getRuntime() - slowTime > 0.5) {
            slowMode = !slowMode;
        }
        time = op.getRuntime();
        //according to blue side left auto next to barrier
        changed = false;
        logger.loopcounter++;
        int up = 0;
        intake.updateIntakeStates();
        turret.updateTurretPositions();
        checker.logCurrentStates();

//        if (xpos > (60) * red - startx) {
//            if (!shared_shipping_hub && red == 1) {
//                changed = true;
//            }
//            if (!alliance_shipping_hub && red == -1) {
//                changed = true;
//            }
//            if (red == 1) {
//                shared_shipping_hub = true;
//                alliance_shipping_hub = false;
//            } else {
//                shared_shipping_hub = false;
//                alliance_shipping_hub = true;
//            }
//            up = 0;
//        } else if (xpos < (60) * red - startx) {
//            if (!shared_shipping_hub && red == -1) {
//                changed = true;
//            }
//            if (!alliance_shipping_hub && red == 1) {
//                changed = true;
//            }
//            if (red == 1) {
//                alliance_shipping_hub = true;
//                shared_shipping_hub = false;
//            } else {
//                alliance_shipping_hub = false;
//                shared_shipping_hub = true;
//            }
//            up = 1;
//        }

        if (changed) {
            if (!checker.getState(BASKET_ARM_REST)) {
                    turret.FlipBasketArmToPosition(.45);
            }
            //according to blue side left auto next to barrier
        }
//        if (checker.getState(SLIDES_EXTENDED)) {
//            autoretractTurret = false;
//            autoAim = false;
//            autoAiming = false;
//        }
        if(autoAiming){
            turret_angle_control_distance=17*118.0/270.0/35.0;
        }
        if(checker.getState(SLIDES_RETRACTING)){
            turret_angle_control_distance=0;
        }
        turret_angle_control_distance -= turretUpNDown / 300;
        if (turret_angle_control_distance > 1) {
            turret_angle_control_distance = 1;
        }
        if (turret_angle_control_distance < 0) {
            turret_angle_control_distance = 0;
        }
        op.telemetry.addData("diSTANCE", turret_angle_control_distance);

        if (startIntake && checker.checkIf(INTAKING)) {
            startIntake();
        }
        else if (checker.getState(SEQUENCING)) {

        }
        else {
            stopIntake();
        }

        if (reverseIntake && checker.checkIf(INTAKE_REVERSING) || checker.getState(INTAKE_REVERSING)) {
            op.telemetry.addData("reversing", "intake");
            reverseIntake(0.7);
        }

        if (flipIntake && time > startTime[3] + .3) {
            startTime[3] = time;
            if (!intake.flipIntake() && checker.checkIf(SLIDES_RETRACTING)) {
                checker.setState(SLIDES_RETRACTING, true);
            }
        }
        if (resetTurret) {
            turret.resetExtension();
            op.sleep(200);
        }
        if (plusTurret) {
            turret.plusExtension();
            op.sleep(200);
        }
        if (turretTurn != 0&&!checker.getState(SLIDES_RETRACTING)) {
            autoAiming = false;
            TurretManualRotation(turretTurn);
        } else if (!checker.getState(SLIDES_RETRACTING) && !autoAiming && time > startTime[8] + 2) {
            turret.stopTurn();
        }
//        if (extendAutoTSE) {
//            startTime[6] = op.getRuntime();
////            TSE.setTseCrServoPower(1.0);
//            isExtending = true;
//            isExtended = true;
//        }
        if (capper && time > startTime[8] + 0.4) {
            capThats();
            startTime[8] = time;
        }
//        if (op.getRuntime() > .147 * 44 + startTime[6]) {
//            checker.setState(SLIDES_EXTENDING, false);
//        }
        if (extendTurret != 0 || checker.getState(SLIDES_RETRACTING)) {
            autoAiming = false;
            TurretManualExtension(extendTurret);
        } else if (time > startTime[8] + 2.0) {
            turret.stopExtend();
            if (outModed) {
                turret.runTurretWithoutEncoder();
                outModed = false;
            }
        } else if (time < startTime[8] + 2.0) {
            outModed = true;
        }

        if (turretUpNDown != 0) {
            autoAiming = false;
            TurretAngleControlRotating(turret_angle_control_distance);
        }

        if (autoAim) {
            if (op.getRuntime() > startTime[2] + 0.5) {
//                autoAiming = !autoAiming;
                startTime[2] = op.getRuntime();
                TurretStop();
            }

        }
        spinCarousel();
        if (autoAiming && !checker.getState(SLIDES_RETRACTING)) {
            fakeAutoAim();
        }
        op.telemetry.addData("autoaiming", autoAiming);
        op.telemetry.addData("up", up);

        if (basket && time > startTime[4] + .3) {
            startTime[4] = time;
            FlipBasket(up);
            if (autoAiming) {
                op.sleep(500);
            }
//            intake.flipIntakeToPosition(0.0);
//            checker.setState(INTAKE_FLIPPING_DOWN, true);
//            op.sleep(200);
            SavePosition(up);
        }
        if (basketArm) {
            if (time > startTime[5] + 0.3) {
                startTime[5] = time;
                turret.FlipBasketArmHigh();
            }
        }

        if (TSEArmUp && time > startTime[12] + 0.2) {
            startTime[12] = time;
            TSE.toggleTSEPosition();
//            intake.flipIntakeToPosition(.79);
            checker.setState(INTAKE_FLIPPING_UP, true);
        }


//        if (extendTSE) {
//            TSE.setTseCrServoPower(1);
//        }
//        if (!extendTSE && !manualretractTSE && !autoretractTSE && !isExtending) {
//            TSE.setTseCrServoPower(0.0);
//        }
//
//        if (manualretractTSE) {
//            TSE.setTseCrServoPower(-1);
//        }
//
//        if (autoretractTSE) {
//            //insert set tse to 0 function
//        }

        /** turret stuff **/


        if (unsave_turret_position) {
            UnsavePosition();
        }

        /** insert basket stuff **/
        /** insert basket stuff **/


//            if (slowDown) {
//                if(slowMode){
//                    slowMode=false;
//                }
//                else{
//                    slowMode=true;
//                }
//
//                slowMode = !slowMode;
//                op.sleep(150);
//            }

        /** add stuff u want to do with intake when switch is on HERE **/
        if ((checker.getState(SEQUENCING) || intake.isSwitched() && !checker.getState(SLIDES_RETRACTING))) {
            op.telemetry.addData("el button", "is clicked");
            isFlipping = true;
            if (!checker.getState(SEQUENCING)) {
                startTime[0] = op.getRuntime() + 9;
                startTime[1] = op.getRuntime() + 10;
                turret.FlipBasketToPosition(.89);
                checker.setState(BASKET_TRANSFER, true);
                intake.startIntake();
            }
            checker.setState(SEQUENCING, true);
            turret.FlipBasketArmToPosition(0.00);
            checker.setState(BASKET_ARM_REST, true);
            if (checker.checkIf(INTAKE_FLIPPING_UP) && checker.getState(INTAKE_DOWN)) {
                intake.spinIntake(0.2);
                checker.setState(INTAKING, true);
                intake.flipIntakeToPosition(0.72);
                turret.FlipBasketToPosition(.89);
                isBall = intake.isBall();
                startTime[0] = op.getRuntime();
                startTime[1] = op.getRuntime() + 10;
            } else if (!checker.getState(INTAKE_FLIPPING_UP) && checker.getState(INTAKE_DOWN)) {
                checker.setState(SLIDES_RETRACTING, true);
            }
            op.telemetry.addData("transferring", !checker.getState(TRANSFERRING));
            op.telemetry.addData("flipping", !checker.getState(INTAKE_FLIPPING_DOWN));
            op.telemetry.addData("straight", checker.getState(TURRET_STRAIGHT));
            op.telemetry.addData("extend", !checker.getState(SLIDES_EXTENDED));
            op.telemetry.addData("raise", !checker.getState(TURRET_RAISED));
            op.telemetry.addData("basket", checker.getState(BASKET_TRANSFER));
            op.telemetry.addData("intakedown", !checker.getState(INTAKE_DOWN));
            if (op.getRuntime() > startTime[0] + 0.5 && checker.getState(INTAKE_UP)&&!checker.getState(TRANSFERRING)) {
               stopIntake();
            }

            if (op.getRuntime() > startTime[0] + 0.8 && checker.getState(INTAKE_UP)&&!checker.getState(TRANSFERRING)) {
                op.telemetry.addData("reversing ", "intake");
                startTime[1] = op.getRuntime();
                if(!isBall) {
                    reverseIntake(0.63);
                    checker.setState(TRANSFERRING, true);
                }
                else{
                    startTime[1]=0.1;
                    reverseIntake(0.53);
                    checker.setState(TRANSFERRING, true);
                }
            }

            if (op.getRuntime() > startTime[1] + 1) {
//                isReversing = false;
//                isFlipping = false;
//                flipped = false;
                stopIntake();
//                FlipBasketToPosition(.6);
//                turret.FlipBasketArmToPosition(.3);
                autoAiming = true;
                checker.setState(TRANSFERRING, false);
                checker.setState(SEQUENCING, false);
            }

        }
        if (autoretractTurret || checker.getState(SLIDES_RETRACTING)) {
            autoAiming = false;
            if (!checker.getState(INTAKE_DOWN)&&abs(startAngle-EncoderChassis.angle)<45) {
//                intake.flipIntakeToPosition(0.0);
            }
            if (!TurretReset(0.5)) {
                checker.setState(SLIDES_RETRACTED, true);
            }
        }

        magnitude = forward;
//        if(ypos<20||angle>30||magnitude<0) {
        drivetrain.moveMultidirectional(magnitude, angleInDegree, turning, slowMode); // It is 0.95, because the robot DCs at full power.
//        }
//        else{
//            drivetrain.setRightMotorVelocities(pow(48-EncoderChassis.ypos,1/2.0)/4.46*30*29.8 + angle *20);
//            drivetrain.setLeftMotorVelocities(pow(48-EncoderChassis.ypos,1/2.0)/4.46*30*29.8- angle *20);
//        }
    }

    public void mazeTeleopLoop() {


        /** gamepad 1**/
        float forward = -op.gamepad1.left_stick_y;
        float strafe = -op.gamepad1.left_stick_x; //remove dis boi son
        float turning = -op.gamepad1.right_stick_x;


        magnitude = forward;
        drivetrain.moveMultidirectionalMecanum(0.9, strafe, forward, turning); // It is 0.95, because the robot DCs at full power.
    }

//    public void autoAim(double[][] turret_saved_positions, int red) {
//        double angle = 180 - Math.atan2(-(turret_saved_positions[1][0] - xpos), (turret_saved_positions[1][1] - ypos)) * 180 / PI - VSLAMChassis.angle;
//        angle %= 360;
//        if (angle > 180) {
//            angle -= 360;
//        }
//        if (angle < -180) {
//            angle += 360;
//        }
//        turret.TurretRotate(angle);
//        op.telemetry.addData("angle", angle);
//        op.telemetry.addData("trutx", turret_saved_positions[1][0]);
//        op.telemetry.addData("truty", turret_saved_positions[1][1]);
//
//        double turret_angle_control_pos = Math.atan2(turret_saved_positions[1][2], Math.sqrt(Math.pow(xpos - turret_saved_positions[1][0] * red, 2) + Math.pow(ypos - turret_saved_positions[1][1], 2)));
//        turret.AutoAngleControlRotating(turret_angle_control_pos);
//    }

    public void fakeAutoAim() {
        double angle = -60;
//        checker.setState(SLIDES_RETRACTING, false);
        if(abs(startAngle-EncoderChassis.angle)%360<45) {
            if(!flipped) {
                flipBasketArmToPosition(0.55);
                checker.setState(BASKET_ARM_SHARED, true);
                flipped=true;
                shareFlipTime = op.getRuntime();
                flipIntakeToPosition(0.0);
            }
            if (op.getRuntime() - shareFlipTime > 0.3) {
                if (!isBall) {
                    turret.TurretRotate(turret.turret_saved_positions[0][0][1] * isBlue);
                    turret.AutoAngleControlRotating(17);
                    if (abs(turret.turret_saved_positions[0][0][1] * isBlue - turret.rotatePosition) < 200 || isBlue == -1) {
                        turret.turretExtendo(turret.turret_saved_positions[0][0][0]);
                        checker.setState(SLIDES_EXTENDED, true);
                    }
                } else {
                    turret.TurretRotate(turret.turret_saved_positions[0][1][1] * isBlue);
                    turret.AutoAngleControlRotating(0);
                    if (abs(turret.turret_saved_positions[0][1][1] * isBlue - turret.rotatePosition) < 200 || isBlue == -1) {
                        turret.turretExtendo(turret.turret_saved_positions[0][1][0]);
                        checker.setState(SLIDES_EXTENDED, true);
                    }
                }
            }
        }
        else{
            if(time-shareFlipTime>3.0&&checker.getState(BASKET_ARM_REST)&&!flipped) {
                flipBasketArmToPosition(0.25);
                checker.setState(BASKET_ARM_ALLIANCE, true);
                shareFlipTime=time;
                flipped = true;
            }
            if(checker.getState(INTAKE_DOWN)){
                flipIntakeToPosition(0.76);
            }
            if(time-shareFlipTime>0.1){
                turret.TurretSlotate(turret.turret_saved_positions[0][2][1]*isBlue);
                turret.AutoAngleControlRotating(0);
                turret.turretExtendo(turret.turret_saved_positions[0][2][0]);
            }
        }
    }

    public void rotateToPosition(double targetAngle) {
        turret.rotateToPosition(targetAngle);
    }

    public void capThats() {
        turret.capBasket();
    }

    public void updateTurretPositions() {
        turret.updateTurretPositions();
    }

    public void TurretExtend(double height_inches, double extension_inches, double power) {
        turret.TurretExtend(height_inches, extension_inches, power);
    }

    public void TurretExtendSimple(int extension_inches, double power) {
        turret.TurretExtendSimple(extension_inches, power);
    }

    public void TurretRotate(double targetAngle) {
        TurretRotate(targetAngle);
    }

    public void BasketArmFlipLowExtend(double power) {
        turret.BasketArmFlipLowExtend(power);
    }

    public void BasketArmFlipHighExtend(double power) {
        turret.BasketArmFlipHighExtend(power);
    }

    public void TurretAngleControlRotating(double target_point) {
        turret.TurretAngleControlRotating(target_point);
    }

    public void TurretSlidesToPosition(double x, double y, double z, double power, boolean retract) {
        turret.TurretSlidesToPosition(x, y, z, power, retract);
    }

    public void setMotorPowers(double power) {
        drivetrain.setMotorPowers(power);
    }

    public void TurretManualRotation(double rotation) {
        turret.TurretManualRotation(rotation);
    }

    public void TurretManualExtension(double turret_extension) {
        turret.TurretManualExtension(turret_extension);
    }

    public void turretManualElevation(double elevation) {
        turret.TurretManualElevation(elevation);
    }

    public void TurretManualFlip() {
        turret.TurretManualFlip();
    }

    public void FlipBasketArmLow() {
        turret.FlipBasketArmLow();
    }

    public void FlipBasketToPosition(double torget) {
        turret.FlipBasketToPosition(torget);
    }

    public void FlipBasketArmHigh() {
        turret.FlipBasketArmHigh();
    }

    public void FlipBasketArmToPosition(double torget) {
        turret.FlipBasketArmToPosition(torget);
    }

    public void FlipBasket(int up) {
        turret.FlipBasket(up);
    }

    public void SavePosition(int up) {
        turret.SavePosition(up);
    }

    public void UnsavePosition() {
        turret.UnsavePosition();
    }

    public boolean TurretReset(double power) {
        return turret.TurretReset(power);
    }

    public void TurretStop() {
        turret.TurretStop();
    }


    public void AngleControlRotation(double degrees) {
        turret.AngleControlRotating(degrees);
    }

    public void spinCarousel() {
        rotation.spinCarousel();
    }

    public void spinCarouselAutonomousBlue() {
        rotation.spinCarouselAutonomousBlue();
    }

    public void spinCarouselAutonomousRed() {
        rotation.spinCarouselAutonomousRed();
    }

    public void startIntake() {
        intake.startIntake();
    }

    public void reverseIntake(double power) {
        intake.reverseIntake(power);
    }

    public void stopIntake() {
        intake.stopIntake();
    }

    public void setPosition(float xPosition, float yPosition, float newAngle) {
        drivetrain.setPosition(xPosition, yPosition, newAngle);
    }

    public void aimHigh() {
        turret.aimHigh();
    }

    public double[] track() {
        return drivetrain.track();
    }//track[0] is y value, track[1] is x value, and track[2] is angle

    public void goToPosition(int direction, double yPosition, double xPosition, double newAngle, double power) {
        drivetrain.goToPosition(direction, yPosition, xPosition, newAngle, power);
    }

    public boolean goToPositionTeleop(int direction, double yPosition, double xPosition, double power) {
        return drivetrain.goToPositionTeleop(direction, yPosition, xPosition, power);
    }

    public void flipIntakeToPosition(double torget) {
        intake.flipIntakeToPosition(torget);
    }

    public void turnInPlace(double target, double power) {
        drivetrain.turnInPlace(target, power);
    }

    public void toggleTSEPosition() {
        TSE.toggleTSEPosition();
    }

    public void tseToPosition(double position) {
        TSE.tseToPosition(position);
    }

    /**
     * LEDs
     **/
    public void ledAmber(int led_number) {
        led_bank.LedAmber(led_number);
    }

    public void ledGreen(int led_number) {
        led_bank.LedGreen(led_number);
    }

    public void ledRed(int led_number) {
        led_bank.LedRed(led_number);
    }

    public void ledOff(int led_number) {
        led_bank.LedOff(led_number);
    }
}