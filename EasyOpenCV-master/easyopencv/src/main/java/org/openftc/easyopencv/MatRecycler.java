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

import android.util.Log;

import org.opencv.core.Mat;

import java.util.concurrent.ArrayBlockingQueue;

/*
 * A utility class for managing the re-use of Mats
 * so as to re-use already allocated memory instead
 * of constantly allocating new Mats and then freeing
 * them after use.
 */
class MatRecycler
{
    private RecyclableMat[] mats;
    private ArrayBlockingQueue<RecyclableMat> availableMats;

    MatRecycler(int num)
    {
        mats = new RecyclableMat[num];
        availableMats = new ArrayBlockingQueue<>(num);

        for(int i = 0; i < mats.length; i++)
        {
            mats[i] = new RecyclableMat(i);
            availableMats.add(mats[i]);
        }
    }

    synchronized RecyclableMat takeMat() throws InterruptedException
    {
        if(availableMats.size() == 0)
        {
            throw new RuntimeException("All mats have been checked out!");
        }

        RecyclableMat mat = availableMats.take();
        mat.checkedOut = true;
        return mat;
    }

    synchronized void returnMat(RecyclableMat mat)
    {
        if(mat != mats[mat.idx])
        {
            throw new IllegalArgumentException("This mat does not belong to this recycler!");
        }

        if(mat.checkedOut)
        {
            mat.checkedOut = false;
            availableMats.add(mat);
        }
        else
        {
            throw new IllegalArgumentException("This mat has already been returned!");
        }
    }

    class RecyclableMat extends Mat
    {
        private int idx = -1;
        private volatile boolean checkedOut = false;

        private RecyclableMat(int idx)
        {
            this.idx = idx;
        }
    }
}