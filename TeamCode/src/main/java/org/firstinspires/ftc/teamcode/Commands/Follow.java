package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;

public class Follow {
    boolean initial = true;
    boolean done = false;
    Timer timer = new Timer();
    public boolean isDone(){
        return done;
    }
    public int run(FollowerHandler followerHandler, PathChain path, long time, double speed){
        if (initial) {
            timer.setWait(time);
            followerHandler.getFollower().followPath(path,speed,true);
            initial = false;
            done = false;
        }
        if (timer.doneWaiting()){
            done = true;
            initial = true;
            return 1;
        }
        return 0;
    }

}
