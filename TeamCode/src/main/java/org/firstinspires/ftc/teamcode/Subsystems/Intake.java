package org.firstinspires.ftc.teamcode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Sorting.BallDetector;

@Config
public class Intake {
    public enum States {
        INTAKE,
        SHOOTING,
        OFF,
        EJECT,
    }


    public States currentState = States.OFF;
    public static double intakePower = 1;
    public static double shootingPower = 1;
    public static double ejectPower = -1;
    public static long intakeFullTime = 400;
    Timer intakeFullTimer = new Timer();
    public BallDetector getBallDetector(){
        return ballDetector;
    }


    public void setState(States newStates){
        if (newStates == States.INTAKE && currentState != States.INTAKE){
            intakeFullTimer.setWait(intakeFullTime);
        }
        currentState = newStates;
    }


    DcMotor intakeMotor;
    DcMotor transferMotor;
    BallDetector ballDetector = new BallDetector();
    public States getState() {
        return currentState;
    }
    public void flipState(){
        switch (currentState){
            case INTAKE:
            case EJECT:
                currentState = States.OFF;
                break;
            case OFF:
                currentState = States.INTAKE;
                break;
        }
    }


    public void initiate(HardwareMap hardwaremap){
        transferMotor = hardwaremap.dcMotor.get("transfer");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor = hardwaremap.dcMotor.get("intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ballDetector.initiate(hardwaremap);
    }
    public LimeLight.BallColors prevBall = LimeLight.BallColors.NONE;
    public void update(){
        if (currentState == States.INTAKE){
            if (ballDetector.getBallColor() == LimeLight.BallColors.NONE){
                intakeFullTimer.setWait(intakeFullTime);
            }
            if (prevBall == LimeLight.BallColors.NONE && ballDetector.getBallColor() != LimeLight.BallColors.NONE){
                intakeFullTimer.setWait(intakeFullTime);
            }
            if (prevBall == ballDetector.getBallColor() && intakeFullTimer.doneWaiting()){
                currentState = States.OFF;
            }
        }
        prevBall = ballDetector.getBallColor();
        switch(currentState){
            case INTAKE:
                intakeMotor.setPower(intakePower);
                transferMotor.setPower(intakeMotor.getPower());
                break;
            case OFF:
                intakeMotor.setPower(0);
                transferMotor.setPower(intakeMotor.getPower());
                break;
            case EJECT:
                intakeMotor.setPower(ejectPower);
                transferMotor.setPower(intakeMotor.getPower());
                break;
            case SHOOTING:
                intakeMotor.setPower(shootingPower);
                transferMotor.setPower(intakeMotor.getPower());
                break;
        }
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("INTAKE ------");
        telemetry.addData("Power",intakeMotor.getPower());
        telemetry.addData("State",currentState);
        ballDetector.status(telemetry);
    }
}


