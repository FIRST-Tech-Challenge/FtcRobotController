package org.firstinspires.ftc.teamcode.util.Other;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class WebcamSensorTypeValue implements DynamicTypeValue<WebcamName> {
    private WebcamName value;

    public WebcamSensorTypeValue(WebcamName value) {
        this.value = value;
    }

    @Override
    public WebcamName getValue() {
        return value;
    }

    @Override
    public void setValue(WebcamName value) {
        this.value = value;
    }
}
