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
    public static double downPos = 0.32;
    public static double upPos = 0.14;
    public static double downMax = downPos;
    public static double upMax = upPos;
    public static double defaultPos = (upPos + downPos) / 2;
    public boolean on = true;
    public static double maxDistance = 85;
    public static double a = 0;
    public static double b = 0.01;
    public static double c = 0.4;
    public static double offset = 0;
    Servo hood;
    public void setState(States state){
        this.state = state;
    }
    public double calculatePos(){
        //Relationship Here
        double distance = ShootingWhileMoving.getDistance();
        if (distance > maxDistance){
            return upPos;
        }
        return (a * Math.pow(distance,2)) + (b * distance) + c + downPos;
    }
    public void initiate(HardwareMap hardwareMap){
        hood = hardwareMap.servo.get("hood");
    }
    public double manualPos = 0.5;
    double targetPos = defaultPos;
    public void update(){
        switch (state){
            case RESTING:
                targetPos = downPos - offset;
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
