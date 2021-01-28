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

package teamcode.test.revextensions2;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;


/***
 * This class manages hotswapping of the hardwareMap
 */

class RevExtensions2
{
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
    public static void setupOpModeListenerOnStartRobot(Context context, AnnotatedOpModeManager manager)
    {
        /*
         * Because this is called every time the robot is "restarted", one
         * would think that we should have a boolean to check whether we've
         * already registered this listener to prevent registering duplicate
         * listeners. However, the OpModeManager that we register with
         * is a child of FtcEventLoop, and the EventLoop is re-created every
         * time the robot is "restarted". So thus, we do actually need to
         * register the listener every time this method is called.
         */
        Utils.getOpModeManager().registerListener(opModeNotifications);
    }

    private static class OpModeNotifications implements OpModeManagerNotifier.Notifications
    {
        @Override
        public void onOpModePreInit(OpMode opMode)
        {
            /*
             * We only hotswap the hardware map if this is NOT the
             * "default" (STOP ROBOT) OpMode
             */
            if(!(opMode instanceof OpModeManagerImpl.DefaultOpMode))
            {
                RobotLog.dd("RevExtensions2", "Hotswapping hardware map");
                Utils.hotswapHardwareMap();
            }
        }

        @Override
        public void onOpModePreStart(OpMode opMode)
        {

        }

        @Override
        public void onOpModePostStop(OpMode opMode)
        {
            /*
             * We only deswap the hardware map if this is NOT the
             * "default" (STOP ROBOT) OpMode
             */
            if(!(opMode instanceof OpModeManagerImpl.DefaultOpMode))
            {
                RobotLog.dd("RevExtensions2", "Deswapping hardware map");
                Utils.deswapHardwareMap();
            }
        }
    }
}
