package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Drivetrain {
    public enum Mode {
        FIELD,
        ROBOT
    }
    public static double flP = 1;
    public static double frP = 1;
    public static double blP = 1;
    public static double brP = 1;
    public static double yawOffset = 0;
    public static double redOffset =0;
    public static double blueOffset = 180;

    Mode mode = Mode.ROBOT;
    double yaw = 0;
    Lights.TeamColors team = Lights.TeamColors.RED;
    public void setTeam(Lights.TeamColors team){
        this.team = team;
    }
    public double teamOffset(){
        switch (team){
            case RED:
            case NONE:
                return redOffset;
            case BLUE:
                return blueOffset;
        }
        return redOffset;
    }
    public void setHeading(double yaw){
        this.yaw = yaw + Math.toRadians(yawOffset + teamOffset());
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void flipMode() {
        switch (mode) {
            case FIELD:
                mode = Mode.ROBOT;
                break;
            case ROBOT:
                mode = Mode.FIELD;
                break;
        }
    }

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;


    public void initiate(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        backRightMotor = hardwareMap.dcMotor.get("br");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double x, double y, double rx) {


        y = -y;

        if (mode == Mode.FIELD){
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
            double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            //Make the formula work!
            x = rotX;
            y = rotY;
        }


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        frontLeftMotor.setPower(frontLeftPower * flP);
        backLeftMotor.setPower(backLeftPower * blP);
        frontRightMotor.setPower(frontRightPower * frP);
        backRightMotor.setPower(backRightPower * brP);
    }

    public void status(Telemetry telemetry){
        telemetry.addData("DT Yaw Degrees", Math.toDegrees(yaw));
        telemetry.addData("DT Mode",mode);
        telemetry.addData("DT Team",team);
        telemetry.addData("FL power",frontLeftMotor.getPower());
        telemetry.addData("FR power",frontRightMotor.getPower());
        telemetry.addData("BL power",backLeftMotor.getPower());
        telemetry.addData("BR power",backRightMotor.getPower());
    }


}

