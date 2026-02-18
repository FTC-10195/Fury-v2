package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;

public class KalmanFilters {
    public boolean on = true;
    public Pose calculate(FollowerHandler followerHandler){
        if (!on){
            return followerHandler.getFollower().getPose();
        }
        //Placeholder
        return followerHandler.getFollower().getPose();

    }
}
