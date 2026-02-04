package org.firstinspires.ftc.teamcode.Commands;

public class Stopwatch {
    long timeSnapshot = System.currentTimeMillis();
    long timePassed = 0;
    boolean initial = true;
    public long getTimePassed(){
        return timePassed;
    }
    public void reset(){
        initial = true;
    }

    public void run(){
        if (initial){
            timeSnapshot = System.currentTimeMillis();
            initial = false;
        }
        timePassed = System.currentTimeMillis() - timeSnapshot;
        return;
    }
}
