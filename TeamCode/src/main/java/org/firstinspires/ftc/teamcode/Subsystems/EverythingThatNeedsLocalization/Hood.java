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
    public static double startPos = 0.5;
    Servo hood;
    public double calculatePos(){
        //Relationship Here
        double distance = ShootingWhileMoving.getDistance();
        return 0.5;
    }
    public void initiate(HardwareMap hardwareMap){
        hood = hardwareMap.servo.get("hood");
    }
    public double manualPos = 0.5;
    double targetPos = startPos;
    public void update(){
        switch (state){
            case RESTING:
                targetPos = startPos;
                break;
            case ADJUST:
                targetPos = calculatePos();
                break;
            case MANUAL:
                targetPos = manualPos;
                break;
        }
    }

}
