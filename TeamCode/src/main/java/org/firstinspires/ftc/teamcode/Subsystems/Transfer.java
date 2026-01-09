package org.firstinspires.ftc.teamcode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {
    public enum States {
        ON,
        OFF,
        OUTTAKE,
    }


    public States currentState = States.OFF;
    public static double transferPower = 1;
    public static double ejectPower = -1;


    public void setState(States newStates){
        currentState = newStates;
    }


    DcMotor transferMotor;
    public States getState() {
        return currentState;
    }


    public void initiate(HardwareMap hardwaremap){
        transferMotor = hardwaremap.dcMotor.get("transfer");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update(){
        switch(currentState){
            case ON:
                transferMotor.setPower(transferPower);
                break;
            case OFF:
                transferMotor.setPower(0);
                break;
            case OUTTAKE:
                transferMotor.setPower(ejectPower);
                break;
        }
    }
}


