package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Flywheel {
    public enum States {
        SPINNING,
        RESTING,
    }
    public enum Zone{
        FAR,
        NEAR
    }
    Zone zone = Zone.NEAR;
    public boolean isReady = false;
    public static double manualVelocityGain = 50;
    public static long waitTime = 500;
    public static double farVelocityIncrease = 250;
    public static double defaultVelocity = 1550;
    public static double farDistance = 110;
    public static double kP = 0.0012;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00041;
    public static double tolerance = 100;
    public static double maxPower = 1;
    public double currentVelocity = 0.0000;
    public static double rMod = 1;
    public static double lMod = -1;
    private double targetVelocity = defaultVelocity;
    private double manualVelocity = 0;

    Timer overideTimer = new Timer();

    PIDFController pidfController = new PIDFController(new PIDFCoefficients(kP,kI,kD,kF));



    public States getState() {
        return currentState;

    }


    public States currentState = States.RESTING;
    DcMotorEx flywheel;
    DcMotorEx flywheel2;
    public void initiate(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void setState(States newState) {
        if (newState == States.SPINNING && currentState != States.SPINNING) {
            overideTimer.setWait(waitTime);
        }
        currentState = newState;
    }
    public void flipState(){
        switch (currentState){
            case RESTING:
                currentState = States.SPINNING;
                break;
            case SPINNING:
                currentState = States.RESTING;
                break;
        }
    }
    double prevPos = 0;
    double posDifference = 0;
    long timeSnapshot = System.currentTimeMillis();
    double timeDifference = 0;
    double power = 0;
    static double distance = 0;
    public void calculateZone(Pose robotPose, Lights.TeamColors color){
        calculateZone(robotPose,Turret.getGoal(color));
    }
    private void calculateZone(Pose robotPose, Pose goal){
        double deltaX = goal.getX() - robotPose.getX();
        double deltaY = goal.getY() - robotPose.getY();
        distance = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
        if (distance > farDistance){
            zone = Zone.FAR;
            return;
        }
        zone = Zone.NEAR;
    }
    public void add(){
        manualVelocity += manualVelocityGain;
    }
    public void sub(){
        manualVelocity -= manualVelocity;
    }

    public void update() {
        if (zone == Zone.FAR){
            targetVelocity = defaultVelocity + manualVelocity + farVelocityIncrease;
            return;
        }
        targetVelocity = defaultVelocity + manualVelocity;

        pidfController.setCoefficients((new PIDFCoefficients(kP,kI,kD,0)));
        pidfController.setTargetPosition(targetVelocity);
        pidfController.updatePosition(currentVelocity);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        posDifference = flywheel.getCurrentPosition() - prevPos;
        timeDifference = (double) ((System.currentTimeMillis() - timeSnapshot) / 1000.00); //Convert to seconds

        timeSnapshot = System.currentTimeMillis();
        prevPos = flywheel.getCurrentPosition();

        currentVelocity = posDifference / (timeDifference) * lMod;


      switch (getState()){
          case RESTING:
              power = 0;
              isReady = false;
              break;
          case SPINNING:
            //  power = maxPower;
              power = pidfController.run() + (kF * targetVelocity);
              if (overideTimer.doneWaiting() || Math.abs(targetVelocity - currentVelocity) < tolerance){
                  isReady = true;
              }
              break;
      }

        if (Math.abs(power) > maxPower){
            power = Math.signum(power) * maxPower;
        }
        flywheel.setPower(power * rMod);
        flywheel2.setPower(power * lMod);
    }
    public void status (Telemetry telemetry) {
        telemetry.addLine("FLYWHEEL -----------");
        telemetry.addData("Distance",distance);
        telemetry.addData("Calculated velocity", currentVelocity);
        telemetry.addData("Calculated VelocityError", targetVelocity - currentVelocity);
        telemetry.addData("Current Pos", flywheel.getCurrentPosition());
        telemetry.addData("Prev Pos", prevPos);
        telemetry.addData("Pos diff", posDifference);
        telemetry.addData("Flywheel power", power);
        telemetry.addData("flywheelReady",isReady);
    }
    public void updateTelemetryPacket(TelemetryPacket telemetryPacket){
        telemetryPacket.put("VelocityError",targetVelocity - currentVelocity);
    }
}
