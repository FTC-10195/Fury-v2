package org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Hood {
    public enum States{
        ADJUST,
        RESTING,
        MANUAL
    }
    States state = States.RESTING;
    public static double defaultPos = 0.7;
    public static double zeroPos = 0.5;
    public boolean on = true;
    public static double maxDistance = 85;
    public static double a = 0;
    public static double b = 0.01;
    public static double c = 0.4;
    Servo hood;
    public void setState(States state){
        this.state = state;
    }
    public double calculatePos(){
        //Relationship Here
        double distance = ShootingWhileMoving.getDistance();
        if (distance > maxDistance){
            return defaultPos;
        }
        return (a * Math.pow(distance,2)) + (b * distance) + c;
    }
    public void initiate(HardwareMap hardwareMap){
        hood = hardwareMap.servo.get("hood");
    }
    public double manualPos = 0.5;
    double targetPos = defaultPos;
    public void update(){
        switch (state){
            case RESTING:
                targetPos = zeroPos;
                break;
            case ADJUST:
                targetPos = calculatePos();
                break;
            case MANUAL:
                targetPos = manualPos;
                break;
        }
        if (!on) {
            targetPos = defaultPos;
        }
        hood.setPosition(targetPos);
    }

}
