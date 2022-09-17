/*
 * Copyright (c) 2019 OpenFTC Team
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

package org.openftc.easyopencv;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.Stack;

public class LIFO_OpModeCallbackDelegate implements OpModeManagerNotifier.Notifications
{
    private static LIFO_OpModeCallbackDelegate theInstance;

    public static LIFO_OpModeCallbackDelegate getInstance()
    {
        if(theInstance == null)
        {
            theInstance = new LIFO_OpModeCallbackDelegate();

            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).registerListener(theInstance);
        }

        return theInstance;
    }

    private Stack<OnOpModeStoppedListener> stack = new Stack<>();

    public synchronized void add(OnOpModeStoppedListener item)
    {
        stack.push(item);
    }

    public interface OnOpModeStoppedListener
    {
        void onOpModePostStop(OpMode opMode);
    }

    @Override
    public synchronized void onOpModePostStop(OpMode opMode)
    {
        while (!stack.isEmpty())
        {
            stack.pop().onOpModePostStop(opMode);
        }

        theInstance = null;
        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).unregisterListener(theInstance);
    }

    //-----------------------------------------------------------
    // Not used
    //-----------------------------------------------------------

    @Override
    public synchronized void onOpModePreInit(OpMode opMode)
    {

    }

    @Override
    public synchronized void onOpModePreStart(OpMode opMode)
    {

    }
}
