package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Light;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class DrivetrainOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        FollowerHandler followerHandler = new FollowerHandler();
        followerHandler.initiate(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        Lights lights = new Lights();
        lights.initiate(hardwareMap);



        TelemetryPacket telemetryPacket = new TelemetryPacket(true);

        if (isStopRequested()) {
            lights.reset();
            followerHandler.reset();
            return;
        }

        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()) {

            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean X = gamepad1.cross && !previousGamepad1.cross;
            boolean square = gamepad1.square && !previousGamepad1.square;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;

            previousGamepad1.copy(gamepad1);

            if (triangle){
                lights.switchTeamColor();
            }
            if (X) {
                drivetrain.flipMode();
            }

            if (square) {
                followerHandler.flipLock();
            }
            if (RB){
                followerHandler.forceRelocalize(lights.getTeamColor());
            }
            if (LB){
                drivetrain.switchHeadingLock();
            }

            followerHandler.update();
            drivetrain.setHeading(followerHandler.getFollower().getHeading());
            drivetrain.setTeam(lights.getTeamColor());
            lights.update(telemetry);

            if (!followerHandler.isLocked()) {
                drivetrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            drivetrain.status(telemetry);
            followerHandler.status(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        }
    }
}
