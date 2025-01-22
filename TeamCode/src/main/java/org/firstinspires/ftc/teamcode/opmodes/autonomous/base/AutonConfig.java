package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

//@Config
public class AutonConfig {
    public static final String LOG_TAG = AutonConfig.class.getSimpleName();
    public static SonicPIDFController sliderPidController = DeliverySlider.pidController;
    public static int BasketDeliveryPosition = DeliverySlider.BasketDeliveryPosition;
}
