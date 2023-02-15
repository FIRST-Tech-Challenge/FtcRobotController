package org.firstinspires.ftc.teamcode.demoCode;

import static org.firstinspires.ftc.teamcode.demoCode.AnotherClass.stateMachine;


public class ComplexStateMachineDemo {

    static DemoSubsystem system;

    static Boolean demoSubsystemCondition;


    public static void main(String[] args){
        system = new DemoSubsystem();
        demoSubsystemCondition = system.inPosition();
        AnotherClass.driverInput = true;
        while(true){
            stateMachine();
        }
    }
}
