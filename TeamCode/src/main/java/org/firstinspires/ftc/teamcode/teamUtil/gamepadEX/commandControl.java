package org.firstinspires.ftc.teamcode.teamUtil.gamepadEX;

public class commandControl {

    public commandControl(){}
    public void conditionalAction(button.buttonCondition buttonCondition, button.buttonAction buttonAction){
        if(buttonCondition.buttonCondition()){
            buttonAction.buttonAction();
        }
    }

    public void bothConditionAction(button.buttonCondition buttonCondition, button.buttonAction trueAction, button.buttonAction falseAction){
        if(buttonCondition.buttonCondition()){
            trueAction.buttonAction();
        }
        else{
            falseAction.buttonAction();
        }
    }
}
