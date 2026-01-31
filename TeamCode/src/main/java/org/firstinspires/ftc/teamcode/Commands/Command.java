package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Command {
    Flywheel flywheel;
    Gate gate;
    Intake intake;
    FollowerHandler followerHandler;
    public Command(){};
    public Command(Flywheel flywheel, Gate gate, Intake intake, FollowerHandler followerHandler){
        this.flywheel = flywheel;
        this.gate = gate;
        this.intake = intake;
        this.followerHandler = followerHandler;
    }
    Follow follow = new Follow();
    public int runFollow(PathChain path){
        return follow.run(followerHandler,path,0,1);
    }
    public int runFollow(PathChain path, long time){
        return follow.run(followerHandler,path,time,1);
    }
    public int runFollow(PathChain path, long time, double speed){
        return follow.run(followerHandler,path,time,speed);
    }

    Wait wait = new Wait();
    public int runWait(long time){
        return wait.run(time);
    }
    Shoot shoot = new Shoot();
    public int runShoot(){return shoot.run(intake,gate,flywheel);}
    public int runShoot(boolean flywheelStaySpinning){return shoot.run(intake,gate,flywheel, flywheelStaySpinning);}

}
