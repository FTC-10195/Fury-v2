package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Config
public class Slots {
    public enum Slot{
        RIGHT,
        LEFT,
        CENTER
    }
    Servo rightServo;
    Servo leftServo;

    public static double rightDefaultPos = .5;
    public static double leftDefaultPos = .5;
    public static double rightCenterPos = .8;
    public static double leftCenterPos = .8;
    public static long slotWaitTime = 500;
    public static long intakeToCenterTime = 250;

    Command command = new Command();

    Intake intake;
    Transfer transfer;

    public Transfer getTransfer(){
        return transfer;
    }
    public Intake getIntake(){
        return intake;
    }


    private Slot rightSlot = Slot.RIGHT;;
    private Slot leftSlot = Slot.LEFT;

    public void rightToCenter(){
        rightSlot = Slot.CENTER;
        leftSlot = Slot.LEFT;
    }
    public void leftToCenter(){
        rightSlot = Slot.RIGHT;
        leftSlot = Slot.CENTER;
    }
    public void toDefault(){
        rightSlot = Slot.RIGHT;
        leftSlot = Slot.LEFT;
    }

    public int rightToCenterAction(){
        rightSlot = Slot.CENTER;
        leftSlot = Slot.LEFT;
        return command.runWait(slotWaitTime);

    }
    public int leftToCenterAction(){
        leftToCenter();
        return command.runWait(slotWaitTime);
    }
    public int toDefaultAction(){
        toDefault();
        return command.runWait(slotWaitTime);
    }
    public int centerToShootAction(){
        transfer.setState(Transfer.States.ON);
        if (command.runWait(slotWaitTime) == 1){
            transfer.setState(Transfer.States.OFF);
        }
        return command.runWait(slotWaitTime);
    }
    public int intakeToCenterAction(){
        toDefault();
        intake.setState(Intake.States.ON);
        transfer.setState(Transfer.States.OFF);
        return command.runWait(slotWaitTime);
    }
    public int intakeToShootAction(){
        toDefault();
        intake.setState(Intake.States.ON);
        transfer.setState(Transfer.States.ON);
        return command.runWait(slotWaitTime * 2);
    }
    private int slotMovementSequence = 0;
    public int rightToShootAction(){
        switch (slotMovementSequence){
            case 0:
                if (rightSlot != Slots.Slot.CENTER){
                    slotMovementSequence += rightToCenterAction();
                }else{
                    slotMovementSequence++;
                }
                break;
            case 1:
                slotMovementSequence += centerToShootAction();
                if (slotMovementSequence == 2){
                    slotMovementSequence = 0;
                }
                return 1;
        }
        return 0;
    }
    public int leftToShootAction(){
        switch (slotMovementSequence){
            case 0:
                if (leftSlot != Slots.Slot.CENTER){
                    slotMovementSequence += leftToCenterAction();
                }else{
                    slotMovementSequence++;
                }
                break;
            case 1:
                slotMovementSequence += centerToShootAction();
                if (slotMovementSequence == 2){
                    slotMovementSequence = 0;
                }
                return 1;
        }
        return 0;
    }



    public void initiate(HardwareMap hardwareMap, Intake intake, Transfer transfer){
        rightServo = hardwareMap.servo.get("rSort");
        leftServo = hardwareMap.servo.get("lSort");
        this.intake = intake;
        this.transfer = transfer;
    }

    double leftPos;
    double rightPos;

    public void update(){
        rightPos = rightDefaultPos;
        leftPos = leftDefaultPos;

        if (rightSlot == Slot.CENTER){
            rightPos = rightCenterPos;
        }
        if (leftSlot == Slot.CENTER){
            leftPos = leftCenterPos;
        }
        rightServo.setPosition(rightPos);
        leftServo.setPosition(leftPos);
    }
    public void status(Telemetry telemetry){
        telemetry.addData("LeftSlotPos",leftPos);
        telemetry.addData("RightSlotPos",rightPos);
    }

}
