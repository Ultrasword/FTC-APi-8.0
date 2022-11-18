package org.firstinspires.ftc.teamcode.system;

public class Clock {

    // stores time variables
    public static double startTime, endTime, deltaTime;

    public static void start(){
        startTime = getCurrentTime();
        endTime = startTime;
        deltaTime = 0.0;
    }

    public static void update(){
        endTime = startTime;
        startTime = getCurrentTime();
        deltaTime = startTime - endTime;
    }

    public static double getCurrentTime(){
        return (double)(System.nanoTime() / 1000000000);
    }

}
