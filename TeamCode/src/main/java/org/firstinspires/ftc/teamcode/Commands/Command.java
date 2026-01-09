package org.firstinspires.ftc.teamcode.Commands;

public class Command {
    Wait wait = new Wait();
    public int runWait(long time){
        return wait.run(time);
    }
}
