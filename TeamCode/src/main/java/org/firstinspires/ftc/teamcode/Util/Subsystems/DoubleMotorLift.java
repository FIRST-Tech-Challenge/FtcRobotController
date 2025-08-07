package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.PDFLController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class DoubleMotorLift implements Subsystem {


    private static DcMotorEx liftLeft;
    private static DcMotorEx liftRight;

    public static int liftTarget = 0;

    public static double P = 0.0025, D = 0, F = 0.07, L = 0.14;

    public static UniConstants.EncoderReadingFrom readingFrom = UniConstants.EncoderReadingFrom.BOTH ;

    private static final PDFLController pdflController = new PDFLController(P, D, F, L);
    public static double power;

    public static int allowedError = 1;

    private static double currentPosition = 0;

    public static boolean enableController = true;

    public static boolean isAtTarget = false;

    public static Telemetry telemetry;
    private static final int maxCurrentLimit = 5000;


    private static final StateMachine<HeightStates> liftStates = new StateMachine<>(HeightStates.TRANSFER)
            .withState(HeightStates.TRANSFER, (stateRef, name) -> goToLiftTarget(UniConstants.LIFT_TRANSFER))
            .withState(HeightStates.MIDDLE, (stateRef, name) -> goToLiftTarget(UniConstants.LIFT_MIDDLE))
            .withState(HeightStates.BASKET, (stateRef, name) -> goToLiftTarget(UniConstants.LIFT_BASKET))
            .withState(HeightStates.BAR, (stateRef, name) -> goToLiftTarget(UniConstants.LIFT_BAR));





    private DoubleMotorLift() {}




    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        liftLeft = hwmap.get(DcMotorEx.class, UniConstants.SLIDE_MOTOR_LEFT);
        liftRight = hwmap.get(DcMotorEx.class, UniConstants.SLIDE_MOTOR_RIGHT);
        liftLeft.setDirection(UniConstants.SLIDE_MOTOR_LEFT_DIRECTION);
        liftRight.setDirection(UniConstants.SLIDE_MOTOR_RIGHT_DIRECTION);
        resetMotors();

        telemetry = new MultipleTelemetry(opMode.getOpMode().telemetry, FtcDashboard.getInstance().getTelemetry());
        setDefaultCommand(runController());
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {


        if(opMode.getOpModeType() == OpModeMeta.Flavor.TELEOP){
            liftTarget += (int) (Mercurial.gamepad2().leftStickY().state() * 35);
            liftTarget = Math.max(Math.min(liftTarget,2000), 0);
        }

        currentPosition = getCurrentPosition();
        pdflController.setPDFL(P, D, F, L);
        pdflController.setTarget(liftTarget);
        pdflController.update(currentPosition);
        isAtTarget = pdflController.atSetTarget();


        runController();



    }


    @NonNull
    public static Lambda goToLiftTarget(int target){
        return new Lambda("Set Lift Target")
                .setInterruptible(true)
                .setInit(() -> {
                    liftTarget = target;
                })
                .setFinish(() -> isAtTarget);
    }

    @NonNull
    public static Lambda goToLiftTarget(HeightStates state){
        return new Lambda("Set Lift Target")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    switch (state){
                        case TRANSFER:
                            liftTarget = UniConstants.LIFT_TRANSFER;
                            liftStates.setState(HeightStates.TRANSFER);
                            break;
                        case MIDDLE:
                            liftTarget = UniConstants.LIFT_MIDDLE;
                            liftStates.setState(HeightStates.MIDDLE);
                            break;
                        case BASKET:
                            liftTarget = UniConstants.LIFT_BASKET;
                            liftStates.setState(HeightStates.BASKET);
                            break;
                        case BAR:
                            liftTarget = UniConstants.LIFT_BAR;
                            liftStates.setState(HeightStates.BAR);
                            break;
                    }
                })
                .setFinish(() -> isAtTarget);
    }




    public static Lambda runController(){
        return new Lambda("Run Controller")
                .setExecute(() -> {
                    if(enableController){




                        power = pdflController.runPDFL(allowedError);
                        liftLeft.setPower(-power);
                        liftRight.setPower(power);

                    }
                })
                .setFinish(() -> false);
    }

    public static Lambda home() {
        return new Lambda("home-outtake")
                .setInit(() -> {
                    enableController = false;
                    setPower(-1);
                })
                .setFinish(() -> isOverCurrent(maxCurrentLimit))

                .setEnd((interrupted) -> {
                    setPower(0);
                    if (!interrupted) {
                        resetMotors();
                    }
                    enableController = true;
                });
    }

    public static void setPower(double power) {
        enableController = false;
        liftLeft.setPower(-power);
        liftRight.setPower(power);
    }

    public static void resetMotors() {
        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void setLiftTarget(int target){
        liftTarget = target;
    }

    public static void log(UniConstants.loggingState state){
        switch (state){

            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Lift Target ", liftTarget);
                telemetry.addData("Lift Current Read Position ", getCurrentPosition());
                telemetry.addData("Lift Is At Position ", isAtTarget);
                telemetry.addData("Lift State ", liftStates.getState());
                break;
            case EXTREME:
                telemetry.addData("Lift Power Supplied ", power);
                telemetry.addData("Lift Left Encoder ", liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Encoder ", liftRight.getCurrentPosition());
                telemetry.addData("Lift Target ", liftTarget);
                telemetry.addData("Lift Reading From ", readingFrom);
                telemetry.addLine();
                telemetry.addData("Lift Current Left (MA) ", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Lift Current Right (MA) ", liftRight.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Lift Current Read Position ", getCurrentPosition());
                telemetry.addData("Lift Is At Position ", isAtTarget);
                break;



        }

    }
  //Useless
//    public static void setState(HeightStates state){
//        liftStates.setState(state);
//    }

    public static boolean isOverCurrent(double limit) {
        return liftRight.getCurrent(CurrentUnit.MILLIAMPS) > limit || liftLeft.getCurrent(CurrentUnit.MILLIAMPS) > limit;
    }


    public static int getCurrentPosition(){
        return readingFrom == UniConstants.EncoderReadingFrom.LEFT ? liftLeft.getCurrentPosition()   :
                (int) (readingFrom == UniConstants.EncoderReadingFrom.RIGHT ? liftRight.getCurrentPosition() :
                        (double) (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2);
    }

    public static void setController(boolean state){
        enableController = state;
    }


    public enum HeightStates {
        TRANSFER,
        MIDDLE,
        BASKET,
        BAR,
    }


    //Everything down here can be copy and pasted
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    private static final DoubleMotorLift INSTANCE = new DoubleMotorLift();
}
