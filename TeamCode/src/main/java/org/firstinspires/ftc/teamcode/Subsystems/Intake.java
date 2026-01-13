package org.firstinspires.ftc.teamcode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    public enum States {
        ON,
        OFF,
        OUTTAKE,
    }


    public States currentState = States.OFF;
    public static double intakePower = 1;
    public static double ejectPower = -1;


    public void setState(States newStates){
        currentState = newStates;
    }


    DcMotor intakeMotor;
    public States getState() {
        return currentState;
    }


    public void initiate(HardwareMap hardwaremap){
        intakeMotor = hardwaremap.dcMotor.get("intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update(){
        switch(currentState){
            case ON:
                intakeMotor.setPower(intakePower);
                break;
            case OFF:
                intakeMotor.setPower(0);
                break;
            case OUTTAKE:
                intakeMotor.setPower(ejectPower);
                break;
        }
    }
}


