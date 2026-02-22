package org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;

@Config
public class Flywheel {
    public enum States {
        SPINNING,
        PASSIVE
    }
    public enum Mode{
        MANUAL_POWER,
        NORMAL,
        MANUAL_VELOCITY
    }
    public boolean on = true;
    Mode mode = Mode.NORMAL;
    public boolean isReady = false;
    public static double manualVelocityGain = 50;
    public static long waitTime = 1000;
    public static double farVelocity = 1180;
    public static double nearVelocity = 1050;
    public static double defaultVelocity = 850;
    public static double nearDistance = 110;
    public static double farDistance = 125;

    public static double nearTestDistance = 95;
    public static double farTestDistance = 140;
    public static double passivePower = .4;
    public static double autoPassivePower = .35;
    public static double kS = 0.6  ;
    public static double kP = 0.0025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double tolerance = 50;
    public static double maxPower = 1;
    public double currentVelocity = 0.0000;
    public static double rMod = 1;
    public static double lMod = -1;
    private double targetVelocity = defaultVelocity;
    public double velocityOffset = 0;
    public boolean shooting = false;
    public boolean auto = false;
    public static double defaultManualPower = 0.6;
    public static double manualPowerGain = 0.05;
    public static double kV = 0.00013;
    private double manualPower = defaultManualPower;

    Timer overideTimer = new Timer();

    PIDFController pidfController = new PIDFController(new PIDFCoefficients(kP,kI,kD,kF));



    public States getState() {
        return currentState;

    }
    public Mode getMode(){
        return mode;
    }
    public void setMode(Mode mode){
        this.mode = mode;
    }

    public States currentState = States.PASSIVE;
    DcMotorEx flywheel;
    VoltageSensor voltageSensor;
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
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
    double manualVelocity = nearVelocity;
    public static double a = 0;
    public static double b = 3.7;
    public static double c = 702.5;
    public static double minDistance = 0;
    public static double maxThreshold = 100;
    public void setManualVelocity(double v){
        manualVelocity = v;
    }
    public void switchManualVelocity(double v){
        if (mode != Mode.MANUAL_VELOCITY){
            mode = Mode.MANUAL_VELOCITY;
            manualVelocity = v;
            return;
        }
        mode = Mode.NORMAL;
    }
    public void add(){
        velocityOffset += manualVelocityGain;
    }
    public void sub(){
        velocityOffset -= manualVelocityGain;
    }
    public void increaseManualPower(){
        manualPower += manualPowerGain;
    }
    public void decreaseManualPower(){
        manualPower -= manualPowerGain;
    }
    public double bangBangCustom(){
        if (!shooting){
            if (targetVelocity - currentVelocity  > maxThreshold){
                return 1;
            }
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
         //   return (0.0081474 * Math.pow(distance,2)) + (0.611543 * (distance)) + 933.35737;
        //return (3.61531 * distance) + 755.35408;
       // return (0.00108852 * Math.pow(distance,3)) - (0.443267 * Math.pow(distance,2)) + (62.24469 * (distance)) - 1758.29271;
        //return (3.42442 * distance) +RG772.86304;
        if (distance < minDistance){
            return 1250;
        }
        return (3.73037 * distance) + 765.78184;

       // return (a * Math.pow(distance,2)) + (b * distance) + c;
    }


    public void update() {
        distance = ShootingWhileMoving.getDistance();
        targetVelocity = calculateVelocity() + velocityOffset;
        if (mode == Mode.MANUAL_VELOCITY){
            targetVelocity = manualVelocity + velocityOffset;
        }
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
        power = pidfController.run() + (kF * (targetVelocity)) + kS + ((kV * targetVelocity) / (voltageSensor.getVoltage() / 12.0));

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
              if (overideTimer.doneWaiting() || withinTolerance()){
                  isReady = true;
              }
              power = bangBangCustom();
              if (mode == Mode.MANUAL_POWER){
                  power = manualPower;
              }
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
        telemetry.addData("Mode",mode);
        telemetry.addData("Manual Power",manualPower);
        telemetry.addData("Voltage",voltageSensor.getVoltage());
    }
    public void updateTelemetryPacket(TelemetryPacket telemetryPacket){
        telemetryPacket.put("Target Velocity ",targetVelocity);
        telemetryPacket.put("Current Velocity ",currentVelocity);
        telemetryPacket.put("VelocityError",targetVelocity - currentVelocity);
        telemetryPacket.put("Power",Math.abs(flywheel.getPower()));
    }
}
