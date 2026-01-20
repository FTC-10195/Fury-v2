package org.firstinspires.ftc.teamcode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public enum States {
        INTAKE,
        PREPARING_TO_FIRE,
        SHOOTING,
        OFF,
        EJECT,
    }


    public States currentState = States.OFF;
    public static double intakePower = 1;
    public static double ejectPower = -1;


    public void setState(States newStates){
        currentState = newStates;
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
                intakeMotor.setPower(intakePower);
                transferMotor.setPower(intakeMotor.getPower());
                break;
            case PREPARING_TO_FIRE:
                intakeMotor.setPower(intakePower);
                transferMotor.setPower(0);
                break;
        }
    }
}


