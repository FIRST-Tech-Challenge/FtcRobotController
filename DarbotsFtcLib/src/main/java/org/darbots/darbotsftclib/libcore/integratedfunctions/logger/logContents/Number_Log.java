package org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents;

import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogType;

public class Number_Log extends LogContent {
    private Number m_Content;

    public Number_Log(Object content){
        this.setContentValue(content);
    }
    public Number_Log(LogContent log){
        this.setContentValue(log.getContentValue());
    }

    @Override
    public LogType getContentType() {
        return LogType.NUMBER_LOG;
    }

    @Override
    public Object getContentValue() {
        return m_Content;
    }

    @Override
    public void setContentValue(Object contentVal) {
        if(contentVal instanceof Number){
            this.m_Content = (Number) contentVal;
        }
    }
}
