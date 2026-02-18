package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;

@Config
public class BallDetector {
    Timer timer = new Timer();
    public static long intakeTimeThreshold = 400;

    //Will tune thresholds more, hopefully it's this simple (GREEN VS BLUE) but it might not be
    public static int alphaThreshold1 = 250;
    public static int alphaThreshold2 = 250;

    ColorSensor colorSensor;
    ColorSensor colorSensor2;
    public boolean active = true;
    public void initiate(HardwareMap hardwareMap){
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor2 = hardwareMap.colorSensor.get("color2");
    }
    public LimeLight.BallColors getBallColor(){
        if (colorSensor.alpha() < alphaThreshold1 && colorSensor2.alpha() < alphaThreshold2){
            return LimeLight.BallColors.NONE;
        }
        if (colorSensor.green() >= colorSensor.blue()){
            return LimeLight.BallColors.G;
        }
        if (colorSensor2.green() >= colorSensor2.blue()){
            return LimeLight.BallColors.G;
        }

        return LimeLight.BallColors.P;
    }
    public boolean isBall(){
        return getBallColor() != LimeLight.BallColors.NONE;
    }
    public boolean isFull(){
        return timer.doneWaiting();
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("COLOR SENSORS -----------");
        telemetry.addData("Color Sensor Ball", getBallColor());
        telemetry.addData("Color Sensor1 Reading: ","\n Red: " + colorSensor.red() + "\n" +
                "Green: " + colorSensor.green() + "\n" +
                "Blue: " + colorSensor.blue() + "\n" +
                "Alpha: " + colorSensor.alpha());
        telemetry.addData("Color Sensor2 Reading: ","\n Red: " + colorSensor2.red() + "\n" +
                "Green: " + colorSensor2.green() + "\n" +
                "Blue: " + colorSensor2.blue() + "\n" +
                "Alpha: " + colorSensor2.alpha());
    }
}
