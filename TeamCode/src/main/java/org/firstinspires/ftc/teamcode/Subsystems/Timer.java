package org.firstinspires.ftc.teamcode.Subsystems;

public class Timer {
    long timeSnapshot = System.currentTimeMillis();
    long waitTime = 0;
    public Timer(){
        this.waitTime = 0;
        timeSnapshot = System.currentTimeMillis();
    }
    public Timer (long waitTime){
        this.waitTime = waitTime;
        timeSnapshot = System.currentTimeMillis();
    }
    public void setWait(long waitTime){
        this.waitTime = waitTime;
        timeSnapshot = System.currentTimeMillis();
    }
    public long getStartTime(){
        return timeSnapshot;
    }
    public long getTimePassed(){
        return System.currentTimeMillis() - timeSnapshot;
    }
    public long getWaitTime(){
        return waitTime;
    }
    public void add(long time){
        waitTime += time;
    }
    public boolean doneWaiting(){
        return System.currentTimeMillis() - timeSnapshot > waitTime;
    }
}
