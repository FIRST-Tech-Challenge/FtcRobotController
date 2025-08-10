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

public class IntakeLinkage implements Subsystem {
    public static Servo linkageServo;
    public static Servo downServo;
    public static Servo sideServo;
    public static final IntakeLinkage INSTANCE = new IntakeLinkage();
    public static Telemetry telemetry;

    private IntakeLinkage() { }

    private static final StateMachine<LinkageStates> linkageStates = new StateMachine<>(LinkageStates.IN)
            .withState(LinkageStates.OUT, (stateRef, name) -> linkageOut())
            .withState(LinkageStates.IN, (stateRef, name) -> linkageIn());




    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        linkageServo = hwmap.get(Servo.class, UniConstants.INTAKE_LINKAGE_NAME);
        //clawServo.setDirection(Servo.Direction.REVERSE);


        telemetry = new MultipleTelemetry(opMode.getOpMode().telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //Started
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){

    }


    private static void out(){
        linkageServo.setPosition(UniConstants.INTAKE_LINKAGE_OUT); linkageStates.setState(LinkageStates.OUT);}
    private static void in(){
        linkageServo.setPosition(UniConstants.INTAKE_LINKAGE_IN); linkageStates.setState(LinkageStates.IN); }




    @NonNull
    public static Lambda linkageOut(){
        return new Lambda("Intake Linkage Out")
                .addRequirements(INSTANCE)
                .setInit(IntakeLinkage::out);
    }

    @NonNull
    public static Lambda linkageIn(){
        return new Lambda("Intake Linkage In")
                .addRequirements(INSTANCE)
                .setInit(IntakeLinkage::in);
    }

    @NonNull
    public static Lambda toggleLinkage(){
        return new Lambda("Toggle-Linkage")
                .setInit(() -> {
                        switch (linkageStates.getState()) {
                            case IN:
                                out();
                                break;
                            case OUT:
                                in();
                                break;
                            case TRANSFER:
                                break;
                        }

                });

    }




    public static void log(UniConstants.loggingState state){
        switch (state){

            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Intake Linkage State ", linkageStates.getState());
                break;
            case EXTREME:
                telemetry.addData("Intake Linkage State ", linkageStates.getState());
                telemetry.addLine();
                telemetry.addData("Intake Linkage Target Position ", linkageServo.getPosition());
                break;



        }

    }




    private enum LinkageStates {
        IN,
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