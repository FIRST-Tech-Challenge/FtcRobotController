/*
Copyright (c) 2017 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.internal.system;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;

/**
 * {@link Deadline} enhances {@link ElapsedTime} with an explicit duration.
 * A {@link Deadline} can also be cancelled, causing it to expire earlier than
 * it originally would have.
 */
@SuppressWarnings("WeakerAccess")
public class Deadline extends ElapsedTime
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected final long nsDuration;
    protected       long nsDeadline;
    protected       TimeUnit awaitUnit = TimeUnit.NANOSECONDS;
    protected       long msPollInterval = 125;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public Deadline(long duration, TimeUnit unit)
    {
        nsDuration = unit.toNanos(duration);
        nsDeadline = Misc.saturatingAdd(nsStartTime, nsDuration);
    }

    public void reset()
    {
        super.reset();
        nsDeadline = Misc.saturatingAdd(nsStartTime, nsDuration);
    }

    public void cancel() // a handy and usefully-rememberable synonym
    {
        expire();
    }

    public void expire()
    {
        nsDeadline = nsStartTime;
    }

    public long getDuration(TimeUnit unit)
    {
        return unit.convert(nsDuration, TimeUnit.NANOSECONDS);
    }

    public long getDeadline(TimeUnit unit)
    {
        return unit.convert(nsDeadline, TimeUnit.NANOSECONDS);
    }

    public long timeRemaining(TimeUnit unit)
    {
        long nsRemaining = Math.max(0, nsDeadline - nsNow());
        return unit.convert(nsRemaining, TimeUnit.NANOSECONDS);
    }

    public boolean hasExpired()
    {
        return timeRemaining(TimeUnit.NANOSECONDS) <= 0;
    }

    //----------------------------------------------------------------------------------------------
    // Awaiting while allowing expire() to be called while that is happening.
    // Note that there are probably other synchronization objects whose awaiting
    // we should also provide for here.
    //----------------------------------------------------------------------------------------------

    public boolean await(CountDownLatch latch) throws InterruptedException
    {
        long pollInterval = awaitUnit.convert(msPollInterval, TimeUnit.MILLISECONDS);
        for (;;)
        {
            long waitInterval = Math.min(pollInterval, timeRemaining(awaitUnit));
            if (waitInterval <= 0)
            {
                return false; // deadline expired
            }
            if (latch.await(waitInterval, awaitUnit))
            {
                return true;
            }
        }
    }

    public boolean tryLock(Lock lock) throws InterruptedException
    {
        long pollInterval = awaitUnit.convert(msPollInterval, TimeUnit.MILLISECONDS);
        for (;;)
        {
            long waitInterval = Math.min(pollInterval, timeRemaining(awaitUnit));
            if (waitInterval <= 0)
            {
                return false; // deadline expired
            }
            if (lock.tryLock(waitInterval, awaitUnit))
            {
                return true;
            }
        }
    }

    public boolean tryAcquire(Semaphore semaphore) throws InterruptedException
    {
        long pollInterval = awaitUnit.convert(msPollInterval, TimeUnit.MILLISECONDS);
        for (;;)
        {
            long waitInterval = Math.min(pollInterval, timeRemaining(awaitUnit));
            if (waitInterval <= 0)
            {
                return false; // deadline expired
            }
            if (semaphore.tryAcquire(waitInterval, awaitUnit))
            {
                return true;
            }
        }
    }
}
