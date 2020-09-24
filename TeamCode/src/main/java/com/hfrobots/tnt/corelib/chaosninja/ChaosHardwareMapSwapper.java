/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.hfrobots.tnt.corelib.chaosninja;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import lombok.NonNull;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@SuppressWarnings("unused")
public class ChaosHardwareMapSwapper {
    // Borrowed heavily from OpenFTC's RevExtensions2 (see copyright)

    /*
     * NOTE: We cannot simply pass `new OpModeNotifications()` inline to the call
     * to register the listener, because the SDK stores the list of listeners in
     * a WeakReference set. This causes the object to be garbage collected because
     * nothing else is holding a reference to it.
     */
    private static final OpModeNotifications opModeNotifications = new OpModeNotifications();

    /*
     * By annotating this method with @OpModeRegistrar, it will be called
     * automatically by the SDK as it is scanning all the classes in the app
     * (for @Teleop, etc.) while it is "starting" the robot.
     */
    @OpModeRegistrar
    public static void setupOpModeListenerOnStartRobot(Context context, AnnotatedOpModeManager manager) {
        /*
         * Because this is called every time the robot is "restarted", one
         * would think that we should have a boolean to check whether we've
         * already registered this listener to prevent registering duplicate
         * listeners. However, the OpModeManager that we register with
         * is a child of FtcEventLoop, and the EventLoop is re-created every
         * time the robot is "restarted". So thus, we do actually need to
         * register the listener every time this method is called.
         */
        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance()
                .getRootActivity()).registerListener(opModeNotifications);
    }

    protected static OpModeManagerImpl getOpModeManager() {
        return OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
    }

    protected static HardwareMap getHardwareMap() {
        return getOpModeManager().getHardwareMap();
    }

    protected static void swapHardwareMap() {
        HardwareMap hardwareMap = getHardwareMap();

        swapMotorsForChaotic(hardwareMap);
        swapServosForChaotic(hardwareMap);
    }

    protected static void deswapHardwareMap() {
        HardwareMap hardwareMap = getHardwareMap();

        swapChaoticMotorsForNormal(hardwareMap);
        swapChaoticServosForNormal(hardwareMap);
    }

    protected static void swapMotorsForChaotic(@NonNull final HardwareMap hardwareMap) {

        // Avoid changing while iterating
        List<Map.Entry<String, DcMotor>> motorsToMakeChaotic = new LinkedList<>();

        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            if (entry.getValue() instanceof DcMotorEx) {
                motorsToMakeChaotic.add(entry);
            }
        }

        for (Map.Entry<String, DcMotor> toMakeChaotic : motorsToMakeChaotic) {
            hardwareMap.dcMotor.remove(toMakeChaotic.getKey());
        }

        for (Map.Entry<String, DcMotor> toMakeChaotic : motorsToMakeChaotic) {
            ChaoticMotor chaoticMotor = new ChaoticMotor((DcMotorEx) toMakeChaotic.getValue());

            hardwareMap.dcMotor.put(toMakeChaotic.getKey(), chaoticMotor);

            Log.i(LOG_TAG, String.format("DcMotorEx %s has been made chaotic", toMakeChaotic.getKey()));
        }
    }

    protected static void swapServosForChaotic(@NonNull final HardwareMap hardwareMap) {
        // Avoid changing while iterating
        List<Map.Entry<String, Servo>> servosToMakeChaotic = new LinkedList<>();

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            if (entry.getValue() instanceof DcMotorEx) {
                servosToMakeChaotic.add(entry);
            }
        }

        for (Map.Entry<String, Servo> toMakeChaotic : servosToMakeChaotic) {
            hardwareMap.servo.remove(toMakeChaotic.getKey());
        }

        for (Map.Entry<String, Servo> toMakeChaotic : servosToMakeChaotic) {
            ChaoticServo chaoticMotor = new ChaoticServo(toMakeChaotic.getValue());

            hardwareMap.servo.put(toMakeChaotic.getKey(), chaoticMotor);

            Log.i(LOG_TAG, String.format("Servo %s has been made chaotic", toMakeChaotic.getKey()));
        }
    }

    protected static void swapChaoticMotorsForNormal(@NonNull final HardwareMap hardwareMap) {
        // Avoid changing while iterating
        List<Map.Entry<String, DcMotor>> motorsToMakeNormal = new LinkedList<>();

        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            if (entry.getValue() instanceof ChaoticMotor) {
                motorsToMakeNormal.add(entry);
            }
        }

        for (Map.Entry<String, DcMotor> toMakeNormal : motorsToMakeNormal) {
            hardwareMap.dcMotor.remove(toMakeNormal.getKey());
        }

        for (Map.Entry<String, DcMotor> toMakeNormal : motorsToMakeNormal) {
            if (toMakeNormal.getValue() instanceof ChaoticMotor) {
                DcMotorEx normalMotor = ((ChaoticMotor) toMakeNormal.getValue()).getActualDcMotor();

                hardwareMap.dcMotor.put(toMakeNormal.getKey(), normalMotor);

                Log.i(LOG_TAG, String.format("DcMotorEx %s has been made non-chaotic", toMakeNormal.getKey()));
            } else {
                Log.w(LOG_TAG,
                        String.format("DcMotor in hardware map named '%s' is unexpectedly not chaotic",
                                toMakeNormal.getKey()));
            }
        }
    }

    protected static void swapChaoticServosForNormal(@NonNull final HardwareMap hardwareMap) {
        // Avoid changing while iterating
        List<Map.Entry<String, Servo>> servosToMakeNormal = new LinkedList<>();

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            if (entry.getValue() instanceof ChaoticServo) {
                servosToMakeNormal.add(entry);
            }
        }

        for (Map.Entry<String, Servo> toMakeNormal : servosToMakeNormal) {
            hardwareMap.servo.remove(toMakeNormal.getKey());
        }

        for (Map.Entry<String, Servo> toMakeNormal : servosToMakeNormal) {
            if (toMakeNormal.getValue() instanceof ChaoticServo) {
                Servo normalServo = ((ChaoticServo) toMakeNormal.getValue()).getActualServo();

                hardwareMap.servo.put(toMakeNormal.getKey(), normalServo);

                Log.i(LOG_TAG, String.format("Servo %s has been made non-chaotic", toMakeNormal.getKey()));
            } else {
                Log.w(LOG_TAG,
                        String.format("Servo in hardware map named '%s' is unexpectedly not chaotic",
                                toMakeNormal.getKey()));
            }
        }
    }

    private static class OpModeNotifications implements OpModeManagerNotifier.Notifications {
        @Override
        public void onOpModePreInit(OpMode opMode) {
            /*
             * We only hotswap the hardware map if this is NOT the
             * "default" (STOP ROBOT) OpMode
             */
            if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
                Log.d(LOG_TAG, "Adding chaotic devices to hardware map");
                swapHardwareMap();
            }
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {

        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            /*
             * We only deswap the hardware map if this is NOT the
             * "default" (STOP ROBOT) OpMode
             */
            if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
                Log.d(LOG_TAG, "Removing chaotic devices from hardware map");
                deswapHardwareMap();
            }
        }
    }
}
