package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Components.Turret.basketActuationDown;
import static org.firstinspires.ftc.teamcode.Components.Turret.basketDown;
import static org.firstinspires.ftc.teamcode.Components.Turret.turretDown;
import static org.firstinspires.ftc.teamcode.Components.Turret.turretStraight;
import static org.firstinspires.ftc.teamcode.Components.Turret.turret_saved_positions;
import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.angle;
import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.xpos;
import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.ypos;
import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.tan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.CarouselCR;
import org.firstinspires.ftc.teamcode.Components.ChassisFactory;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.LedColor;
import org.firstinspires.ftc.teamcode.Components.OpenCVMasterclass;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.VSLAMChassis;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;

import java.util.Arrays;

public class Robot {

    private LinearOpMode op = null;
    public final static boolean isCorgi = true;
    boolean shouldIntake = true;
    boolean shouldFlipIntake = false;
    boolean isReversing=false;
    boolean isFlipping = false;
    double flipDelay=.3, reverseDelay=.7;
    double[] startTime = {0,0,0,0,0,0};
    double magnitude;
    double angleInRadian;
    double angleInDegree;
    double power = 0.5;
    double turret_angle_control_distance = 0;
    boolean slowMode = false;
    boolean autoAiming = false;
    boolean shared_shipping_hub = false;
    boolean alliance_shipping_hub = false;
    public static boolean retracting = false;
    double time;
    boolean changed = false;
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
    private LedColor led_bank = null;
    private OpenCVMasterclass openCV = null;
    private tseDepositor TSE = null;

    public Robot(LinearOpMode opMode, BasicChassis.ChassisType chassisType, boolean isTeleop, boolean vuforiaNAVIGATIONneeded ) {
        op = opMode;
        //This link has a easy to understand explanation. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
        drivetrain = ChassisFactory.getChassis(chassisType, op, vuforiaNAVIGATIONneeded, isCorgi);
        rotation = new CarouselCR(op);
        intake = new Intake(op,isTeleop);
//        led_bank = new LedColor(op); //LED has to be declared before calling it
        turret = new Turret(op, led_bank,isTeleop);
        openCV = new OpenCVMasterclass(op);
        TSE = new tseDepositor(op);

    }

    public int BlueElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.BlueTeamElem();
    }
    public int RedElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.RedTeamElem();
    }

    public double[] BlueWarehouseScam() {
        return openCV.BlueWarehouseScam();
    }
    public void stopAllMotors() {
        drivetrain.stopAllMotors();
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
    public void goToPositionWithoutStop(int direction, double yPosition, double xPosition, double power){
        drivetrain.goToPositionWithoutStop(direction,yPosition,xPosition,power);
    }
    public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power, double targetAnglu){
        drivetrain.tripleSplineToPosition(direction, x0,  y0,  x1,  y1,  x2,  y2,  x3,  y3,  x4,  y4,  power,  targetAnglu);
    }
    public void tripleSplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power){
        drivetrain.tripleSplineToPositionHead(direction,  x0,  y0,  x1,  y1,  x2,  y2,  x3,  y3,  x4,  y4,  power);
    }
    public void partOfPolySplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power, double targetAnglu){
        drivetrain.partOfPolySplineToPosition(direction,  x0,  y0,  x1,  y1,  x2,  y2,  x3,  y3,  start,  end,  targetAnglu,  power);
    }
    public void partOfPolySplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power){
        drivetrain.partOfPolySplineToPositionHead(direction,  x0,  y0,  x1,  y1,  x2,  y2,  x3,  y3,  start,  end,  power);
    }
    public void autonomousBarrierPathWarehouseToShared(double barrierX, double barrierY, double power) {
        double targetY = barrierY - 20;
        double targetX = barrierX;
        double []currentPos = {xpos,ypos,angle};
        double maxAngle = 50;
        double positionAngle = atan2(xpos-barrierX,ypos-barrierY);
        if(maxAngle>positionAngle){
            drivetrain.partOfPolySplineToPositionHead(0,currentPos[0],currentPos[1],currentPos[0],currentPos[1],barrierX, barrierY, targetX, targetY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0,currentPos[0],currentPos[1],barrierX, barrierY, targetX, targetY,targetX-10,targetY, true, true, power);
        }
        else{
            double [] position= {(barrierY*tan(50)+currentPos[1]/tan(50)-barrierX+currentPos[0])/(tan(50)+1/tan(50)),((barrierY*tan(50)+currentPos[1]/tan(50)-barrierX+currentPos[0])/(tan(50)+1/tan(50))-barrierY)*tan(50)+barrierX};
            double[] newPosition = {(4*position[0]+barrierX)/5,(4*position[1]+barrierY)/5};
            drivetrain.partOfPolySplineToPositionHead(0,currentPos[0],currentPos[1],currentPos[0],currentPos[1],newPosition[0],newPosition[1],barrierX, barrierY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0,currentPos[0],currentPos[1],newPosition[0],newPosition[1],barrierX, barrierY, targetX, targetY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0,newPosition[0],newPosition[1],barrierX, barrierY, targetX, targetY,targetX-10,targetY, true, true, power);
        }
    }

    public void teleopLoop() {


            /** gamepad 1**/
            float forward = -op.gamepad1.left_stick_y;
            float strafe = -op.gamepad1.left_stick_x; //remove dis boi son
            float turning = -op.gamepad1.right_stick_x;
            boolean startIntake = op.gamepad1.right_bumper;
            boolean reverseIntake = op.gamepad1.y;
            boolean flipIntake = op.gamepad1.left_bumper;
            boolean extendTSE = op.gamepad2.dpad_right;
            boolean manualretractTSE = op.gamepad2.dpad_left;
            boolean autoretractTSE = op.gamepad1.x;
            boolean sequence = op.gamepad1.dpad_up;

            /** gamepad 2**/
            float turretTurn = op.gamepad2.right_stick_x;
            float turretUpNDown = op.gamepad2.left_stick_y;
            float manualretractTurret = op.gamepad2.left_trigger;
            float extendTurret = op.gamepad2.right_trigger;

            boolean autoretractTurret = op.gamepad2.y;
            boolean basketArm = op.gamepad2.right_bumper;
            boolean autoAim = op.gamepad2.left_bumper;
            boolean basket = op.gamepad2.a;
            boolean unsave_turret_position = op.gamepad2.b;



        time=op.getRuntime();
        //according to blue side left auto next to barrier
        changed = false;
        int up =0;
        turret.updateTurretPositions();
        if (xpos > 60) {
            if (!shared_shipping_hub) {
                changed = true;
            }
            shared_shipping_hub = true;
            alliance_shipping_hub = false;
            up = 0;
        }
        else if (xpos < 60) {
            if (!alliance_shipping_hub) {
                changed = true;
            }
            alliance_shipping_hub = true;
            shared_shipping_hub = false;
            up = 1;
        }

        if (changed) {
            //according to blue side left auto next to barrier
        }

        turret_angle_control_distance -= turretUpNDown/300;
        if(turret_angle_control_distance>1){
            turret_angle_control_distance=1;
        }
        if(turret_angle_control_distance<0){
            turret_angle_control_distance=0;
        }
        op.telemetry.addData("diSTANCE",turret_angle_control_distance);

        if (startIntake && shouldIntake) {
            intake.startIntake();
        } else if(isReversing){
        }
        else{
            stopIntake();
        }

        if (reverseIntake && shouldIntake||isReversing) {
            op.telemetry.addData("reversing","intake");
            intake.reverseIntake(0.7);
        }

        if (flipIntake&&time>startTime[3]+.3) {
            startTime[3]=time;
            intake.flipIntake();
        }

        if (turretTurn != 0) {
            TurretManualRotation(turretTurn);
        }
        else{
            turret.stopTurn();
        }

        if (extendTurret != 0 || manualretractTurret != 0) {
            TurretManualExtension(extendTurret, manualretractTurret);
        }
        else{
            turret.stopExtend();
        }

        if (turretUpNDown != 0) {
            TurretAngleControlRotating(turret_angle_control_distance);
        }

        if (autoAim) {
            if (op.getRuntime() > startTime[2] + 0.5) {
                autoAiming = !autoAiming;
                startTime[2] = op.getRuntime();
                TurretStop();
            }

            op.telemetry.addData("robot position", Arrays.toString(new double[]{xpos,ypos,angle}));
        }
        rotation.spinCarousel();
        if (autoAiming) {
            if(shared_shipping_hub) {
                autoAim(turret_saved_positions[0]);
            }
            else{
                autoAim(turret_saved_positions[1]);
            }
        }
        op.telemetry.addData("autoaiming", autoAiming);

        if (basket&&time>startTime[4]+.3) {
            startTime[4]=time;
            FlipBasket(up);
            SavePosition(up);
        }

        if (basketArm) {
            if (time > startTime[5] + 0.3) {
                startTime[5] = time;
                if (shared_shipping_hub) {
                    FlipBasketArmLow();
                }
                else {
                    FlipBasketArmHigh();
                }
            }
        }
        if (extendTSE) {
            TSE.setTseCrServoPower(1);
        }
        if(!extendTSE&&!manualretractTSE&&!autoretractTSE){
            TSE.setTseCrServoPower(0.0);
        }

            if (manualretractTSE) {
                TSE.setTseCrServoPower(-1);
            }

            if (autoretractTSE) {
                //insert set tse to 0 function
            }

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
        if (!intake.isSwitched()) {
            op.telemetry.addData("el button", "is not clicked");
        }
        if (!intake.isSwitched() && turretStraight&&turretDown&&basketDown&&basketActuationDown|| isFlipping ) {
            //
            //
            op.telemetry.addData("startTime 0", startTime[0]);
            op.telemetry.addData("startTime 1", startTime[1]);
            op.telemetry.addData("el button", "is clicked");
            isFlipping = true;
            shouldIntake = false;
            op.telemetry.addData("is intake stopped?", intake.isSwitched());
            if (!shouldFlipIntake) {
                //
                intake.stopIntake();
                op.telemetry.addData("flip ", "da boi");
                intake.flipIntake();
                shouldFlipIntake = true;
                startTime[0] = op.getRuntime();
                startTime[1] = startTime[0] + 1;
            }
            op.telemetry.addData("time", op.getRuntime() - startTime[1]);
            if (op.getRuntime() > startTime[0] + 1 && !isReversing) {
                isReversing = true;
                op.telemetry.addData("reversing ", "intake");
                startTime[1] = op.getRuntime();
                intake.reverseIntake(0.7);
            }
            if (op.getRuntime() > startTime[1] + 0.8) {
                op.telemetry.addData("flippedy do ", "back down");
                intake.stopIntake();
                if (shared_shipping_hub) {
                    FlipBasketArmLow();
                    turret.FlipBasketToPosition(.6);
                }
                else {
                    FlipBasketArmHigh();
                    turret.FlipBasketToPosition(.6);
                }
                shouldFlipIntake = false;
                shouldIntake = true;
                isReversing = false;
                isFlipping = false;
                autoAiming = true;
            }

        }
        if(autoretractTurret||retracting){
            autoAiming = false;
            retracting = TurretReset(0.5);
        }

        magnitude = forward;
        drivetrain.moveMultidirectional(magnitude, angleInDegree, turning, slowMode); // It is 0.95, because the robot DCs at full power.
    }

    public void autoAim (double [][]turret_saved_positions) {
        double angle = Math.atan2((-turret_saved_positions[1][0] + xpos),(turret_saved_positions[1][1]+ypos))*180/PI- VSLAMChassis.angle;
        turret.TurretRotate(angle);
        op.telemetry.addData("angle",angle);
        double turret_angle_control_pos = Math.atan2(turret_saved_positions[1][2], Math.sqrt(Math.pow(xpos - turret_saved_positions[1][0], 2) + Math.pow(ypos - turret_saved_positions[1][1], 2)));
        turret.AutoAngleControlRotating(turret_angle_control_pos);
    }

    public double Turret_Rotation_Position () {
        return turret.Turret_Rotation_Position();
    }
    public void TurretExtend (double height_inches, double extension_inches, double power) {
        turret.TurretExtend(height_inches, extension_inches, power);
    }
    public void TurretExtendSimple (int extension_inches, double power) {
        turret.TurretExtendSimple(extension_inches, power);
    }
    public void TurretRotate(double targetAngle) {
        TurretRotate(targetAngle);
    }
    public void BasketArmFlipLowExtend (double power) {
        turret.BasketArmFlipLowExtend(power);
    }
    public void BasketArmFlipHighExtend (double power) {
        turret.BasketArmFlipHighExtend(power);
    }
    public void TurretAngleControlRotating (double target_point) {
        turret.TurretAngleControlRotating(target_point);
    }
    public void TurretSlidesToPosition (double x, double y, double z, double power) {
        turret.TurretSlidesToPosition(x, y, z, power);
    }
    public void TurretManualRotation(double rotation) {
        turret.TurretManualRotation(rotation);
    }
    public void TurretManualExtension (double turret_extension, double turret_retraction) {
        turret.TurretManualExtension(turret_extension, turret_retraction);
    }
    public void turretManualElevation(double elevation) {
        turret.TurretManualElevation(elevation);
    }
    public void TurretManualFlip () {
        turret.TurretManualFlip();
    }
    public void FlipBasketArmLow () {
        turret.FlipBasketArmLow();
    }
    public void FlipBasketToPosition(double torget){turret.FlipBasketToPosition(torget);}
    public void FlipBasketArmHigh () {
        turret.FlipBasketArmHigh();
    }
    public void FlipBasketArmToPosition(double torget){
        turret.FlipBasketArmToPosition(torget);
    }
    public void FlipBasket (int up) {
        turret.FlipBasket(up);
    }
    public void SavePosition (int up) {
        turret.SavePosition(up);
    }
    public void UnsavePosition () {
        turret.UnsavePosition();
    }
    public boolean TurretReset (double power) {
        return turret.TurretReset(power);
    }
    public void TurretStop () {
        turret.TurretStop();
    }
    public boolean Turret_Extension_Position () {
        return turret.Turret_Extension_Position();
    }

    public void autoAim (double shipping_hub_x, double shipping_hub_y) {
        double []robot_position={xpos,ypos,angle};
        double turret_relative_rotation_angle = robot_position[2] - turret.Turret_Rotation_Position();
        double turret_target_rotation_angle = atan((robot_position[0] - shipping_hub_x)/(robot_position[1] - shipping_hub_y)) + turret_relative_rotation_angle;
        turret.TurretRotate(turret_target_rotation_angle);
    }

    public void spinCarousel() {rotation.spinCarousel();}
    public void spinCarouselAutonomousBlue() { rotation.spinCarouselAutonomousBlue();}
    public void spinCarouselAutonomousRed() { rotation.spinCarouselAutonomousRed();}

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

    public double[] track() {
        return drivetrain.track();
    }//track[0] is y value, track[1] is x value, and track[2] is angle

    public void goToPosition(int direction, double yPosition, double xPosition, double newAngle, double power) {
        drivetrain.goToPosition(direction, yPosition, xPosition, newAngle, power);
    }

    public void turnInPlace(double target, double power) {
        drivetrain.turnInPlace(target, power);
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