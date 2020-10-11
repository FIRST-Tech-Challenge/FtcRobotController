/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.log;


public enum LogLevel{
    FATAL(5),
    ERROR(4),
    WARNING(3),
    INFO(2),
    DEBUG(1);

    private int value = 0;

    private LogLevel(int value) {    //    必须是private的，否则编译错误
        this.value = value;
    }

    public static LogLevel valueOf(int value) {
        switch (value) {
            case 1:
                return DEBUG;
            case 2:
                return INFO;
            case 3:
                return WARNING;
            case 4:
                return ERROR;
            case 5:
                return FATAL;
            default:
                return null;
        }
    }

    public int value() {
        return this.value;
    }
}