package org.firstinspires.ftc.teamcode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    public enum States {
        INTAKE,
        SHOOTING,
        OFF,
        EJECT,
    }


    public States currentState = States.OFF;
    public Flywheel.Zone zone = Flywheel.Zone.NEAR;
    public static double intakePower = 1;
    public static double farZonePower = 0.8;
    public static double nearZonePower = 0.8;
    public static double ejectPower = -1;


    public void setState(States newStates){
        currentState = newStates;
    }
    public void setZone(Flywheel.Zone zone){
        this.zone = zone;
    }


    DcMotor intakeMotor;
    DcMotor transferMotor;
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
    }
    public void update(){
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
                if (zone == Flywheel.Zone.NEAR){
                    intakeMotor.setPower(nearZonePower);
                }else{
                    intakeMotor.setPower(farZonePower);
                }
                transferMotor.setPower(intakeMotor.getPower());
                break;
        }
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("INTAKE ------");
        telemetry.addData("Power",intakeMotor.getPower());
        telemetry.addData("State",currentState);
    }
}


