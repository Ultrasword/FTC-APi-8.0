package org.firstinspires.ftc.teamcode.wrappers;

public class Clock {

    // stores time variables
    public double startTime, endTime, deltaTime = 1;

    public Clock() {}

    public void start(){
        startTime = getCurrentTime();
        endTime = startTime;
        deltaTime = 0.0;
    }

    public void update(){
        endTime = startTime;
        startTime = getCurrentTime();
        deltaTime = startTime - endTime;
    }

    public double getCurrentTime(){
        return (double)(System.nanoTime() / 1000000000);
    }

}
