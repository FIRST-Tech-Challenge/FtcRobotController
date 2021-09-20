package org.firstinspires.ftc.teamcode.bots;


import org.firstinspires.ftc.teamcode.autonomous.AutoDot;

public class BotActionObj {
    private String methodName;
    private String description;
    private boolean isGeo;
    private String returnRef;


    public String getMethodName() {
        return methodName;
    }

    public void setMethodName(String methodName) {
        this.methodName = methodName;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

    public boolean isGeo() {
        return isGeo;
    }

    public void setGeo(boolean geo) {
        isGeo = geo;
    }

    public String getReturnRef() {
        return returnRef;
    }

    public void setReturnRef(String defaultTarget) {
        this.returnRef = defaultTarget;
    }
}
