package org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents;

import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogType;

public class String_Log extends LogContent {
    private String m_Content;
    public String_Log(Object content){
        this.setContentValue(content);
    }
    public String_Log(LogContent log){
        this.setContentValue(log.getContentValue());
    }
    @Override
    public LogType getContentType() {
        return LogType.STRING_LOG;
    }

    @Override
    public Object getContentValue() {
        return m_Content;
    }

    @Override
    public void setContentValue(Object contentVal) {
        this.m_Content = contentVal.toString();
    }
}
