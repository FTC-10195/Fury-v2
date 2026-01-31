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
        PASSIVE
    }
    public enum Zone{
        FAR,
        NEAR
    }
    public boolean on = true;
    Zone zone = Zone.NEAR;
    public boolean isReady = false;
    public static double manualVelocityGain = 50;
    public static long waitTime = 1000;
    public static double farVelocity = 1180;
    public static double defaultVelocity = 1050;
    public static double nearDistance = 110;
    public static double farDistance = 125;

    public static double nearTestDistance = 95;
    public static double farTestDistance = 140;
    public static double passivePower = .25;
    public static double autoPassivePower = .35;
    public static double kP = 0.00049;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00048;
    public static double tolerance = 50;
    public static double maxPower = 1;
    public double currentVelocity = 0.0000;
    public static double rMod = -1;
    public static double lMod = 1;
    private double targetVelocity = defaultVelocity;
    private double manualVelocity = 0;
    public boolean shooting = false;
    public boolean auto = false;

    Timer overideTimer = new Timer();

    PIDFController pidfController = new PIDFController(new PIDFCoefficients(kP,kI,kD,kF));



    public States getState() {
        return currentState;

    }


    public States currentState = States.PASSIVE;
    DcMotorEx flywheel;
    DcMotorEx flywheel2;
    public void initiate(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //  flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
      // flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
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
            case PASSIVE:
                currentState = States.SPINNING;
                break;
            case SPINNING:
                currentState = States.PASSIVE;
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
    public Zone getZone(){
        return zone;
    }
    public void add(){
        manualVelocity += manualVelocityGain;
    }
    public void sub(){
        manualVelocity -= manualVelocityGain;
    }
    public double bangBangCustom(){
        if (!shooting){
            return power;
        }
        if (Math.abs(currentVelocity) > Math.abs(targetVelocity)){
            return power;
        }
        return 1;
    }
    public double bangBang(){
        if (Math.abs(currentVelocity) > Math.abs(targetVelocity)){
            return 0;
        }
        return 1;
    }
    public double spike(){
        if (shooting){
            return 1;
        }
        return power;
    }
    public boolean withinTolerance(){
        return Math.abs(targetVelocity - currentVelocity) < tolerance;
    }
    public double calculateVelocity(){
            return  (0.000020409 * Math.pow(distance,4)) - (0.0102087 * Math.pow(distance,3)) + (1.90697 * Math.pow(distance,2)) - (153.37581 * (distance)) + 5519.88409;
    }


    public void update() {
        targetVelocity = calculateVelocity() + manualVelocity;
      //  targetVelocity = defaultVelocity;

        pidfController.setCoefficients((new PIDFCoefficients(kP,kI,kD,0)));
        pidfController.setTargetPosition(targetVelocity);
        pidfController.updatePosition(currentVelocity);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        posDifference = flywheel.getCurrentPosition() - prevPos;
        timeDifference = (double) ((System.currentTimeMillis() - timeSnapshot) / 1000.00); //Convert to seconds

        timeSnapshot = System.currentTimeMillis();
        prevPos = flywheel.getCurrentPosition();

        currentVelocity = posDifference / (timeDifference) * rMod;


      switch (getState()){
          case PASSIVE:
              power = passivePower;
              if (auto){
                  power = autoPassivePower;
              }
              shooting = false;
              if (!on){
                  power = 0;
              }
              isReady = false;
              break;
          case SPINNING:
            //  power = maxPower;
              power = pidfController.run() + (kF * targetVelocity);
              if (overideTimer.doneWaiting() || withinTolerance()){
                  isReady = true;
              }
              power = bangBangCustom();
             // power = spike();
              break;

      }

        if (Math.abs(power) > maxPower){
            power = Math.signum(power) * maxPower;
        }
        flywheel.setPower(power *
                rMod);
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
