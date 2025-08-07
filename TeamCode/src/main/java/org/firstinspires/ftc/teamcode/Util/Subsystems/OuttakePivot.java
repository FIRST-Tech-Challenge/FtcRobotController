package org.firstinspires.ftc.teamcode.Util.Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class OuttakePivot implements Subsystem {
    public static Servo leftServo;
    public static Servo rightServo;
    public static final OuttakePivot INSTANCE = new OuttakePivot();
    public static Telemetry telemetry;

    private OuttakePivot() { }

    private static final StateMachine<PivotStates> pivotStates = new StateMachine<>(PivotStates.DOWN)
            .withState(PivotStates.DOWN, (stateRef, name) -> pivotUp())
            .withState(PivotStates.UP, (stateRef, name) -> pivotDown());




    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftServo = hwmap.get(Servo.class, UniConstants.OUTTAKE_PIVOT_LEFT_NAME);
        //clawServo.setDirection(Servo.Direction.REVERSE);

        rightServo = hwmap.get(Servo.class, UniConstants.OUTTAKE_PIVOT_RIGHT_NAME);
        telemetry = new MultipleTelemetry(opMode.getOpMode().telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //Started
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){

    }



    private static void up(){leftServo.setPosition(UniConstants.OUTTAKE_CLAW_CLOSED); pivotStates.setState(PivotStates.DOWN);}
    private static void down(){}





    @NonNull
    public static Lambda pivotUp(){
        return new Lambda("Outtake-Close-Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakePivot::up);
    }

    @NonNull
    public static Lambda pivotDown(){
        return new Lambda("Outtake-Open-Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakePivot::down);
    }





    public static void log(UniConstants.loggingState state){
        switch (state){

            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Outtake Claw State ", pivotStates.getState());
//                telemetry.addData("Outtake Side State ", sideStates.getState());

                break;
            case EXTREME:
                telemetry.addData("Outtake Claw State ", pivotStates.getState());
//                telemetry.addData("Outtake Side State ", sideStates.getState());
                telemetry.addData("Outtake Claw Target Position ", leftServo.getPosition());
                telemetry.addData("Outtake Side Target Position ", rightServo.getPosition());
                break;



        }

    }




    private enum PivotStates {
        UP,
        DOWN,
        OUT,
        TRANSFER
    }


    //Everything down here can be copy and pasted
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}