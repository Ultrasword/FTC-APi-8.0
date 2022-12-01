package org.firstinspires.ftc.teamcode.wrappers;

import android.util.Pair;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Stack;

public class LoggingSystem {

    public class LoggingThread extends Thread {
        private LoggingSystem logger;
        private Stack<String> queue;

        private boolean hasCheck = false;

        public LoggingThread(){
            logger = new LoggingSystem();
        }

        public void pushTelemetry(String key, String value){
            hasCheck = true;
            queue.push(String.format("%s: %s", key, value));
        }

        @Override
        public void run()
        {
            try{
                while(!isInterrupted()){
                    if(hasCheck){
                        String line;
                        while((line = queue.remove(0)) != null){
                            logger.logData(line);
                            line = null;
                        }
                        hasCheck = false;
                    }
                    sleep(100);
                }
            }catch (InterruptedException e){}
            logger.closeLog();
        }
    }

    private FileWriter fileWriter;
    private LoggingThread loggingThread;

    public LoggingSystem(){
        Date date = Calendar.getInstance().getTime();
        DateFormat dateFormat = new SimpleDateFormat("yyyy-mm-dd hh:mm:ss");
        try {
            fileWriter = new FileWriter(String.format("%s.txt", dateFormat.format(date)));
        } catch (IOException e) {
            e.printStackTrace();
        }
        loggingThread = new LoggingThread();
        loggingThread.start();
    }

    public void logData(String data) {
        try {
            fileWriter.write(data);
        }catch(IOException e){
            System.out.println(e);
        }
    }

    public void closeLog() {
        try {
            fileWriter.close();
        }catch(IOException e){
            System.out.println(e);
        }
    }


}
