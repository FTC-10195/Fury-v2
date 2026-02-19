package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Autonomous.Far;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Sorting.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.Webcam;

@TeleOp
public class WebcamTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FollowerHandler followerHandler = new FollowerHandler();
        followerHandler.initiate(hardwareMap);
        Flywheel flywheel = new Flywheel();
        flywheel.initiate(hardwareMap);
        Intake intake = new Intake();
        intake.initiate(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        Lights lights = new Lights();
        lights.initiate(hardwareMap);
        lights.setTeamColor(Lights.TeamColors.RED);
        Turret turret = new Turret();
        turret.initiate(hardwareMap);
        turret.setState(Turret.States.RESET);
        Gate gate = new Gate();
        gate.initiate(hardwareMap);
        Sorter sorter = new Sorter();
        sorter.initiate(hardwareMap);
        Hood hood = new Hood();
        hood.initiate(hardwareMap);
        drivetrain.setYaw(followerHandler.getFollower().getHeading());
        lights.setBall(intake.getBallDetector().getBallColor());
        Webcam webcam = new Webcam();
        webcam.initiate(hardwareMap);
        followerHandler.setStartingPose(new Pose(51.806354009077154, 13.978819969742814, Math.toRadians(180)));
        waitForStart();

        TelemetryPacket telemetryPacket = new TelemetryPacket(true);

        if (isStopRequested()) {
            lights.reset();
            followerHandler.end();
            return;
        }

        boolean override = false;

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeIsActive()) {

            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean square = gamepad1.square && !previousGamepad1.square;
            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1;
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1;
            boolean circle = gamepad1.circle && !previousGamepad1.circle;
            boolean options = gamepad1.options && !previousGamepad1.options;
            boolean share = gamepad1.share && !previousGamepad1.share;

            boolean up = gamepad1.dpad_up && !previousGamepad1.dpad_up;
            boolean down = gamepad1.dpad_down && !previousGamepad1.dpad_down;
            boolean left = gamepad1.dpad_left && !previousGamepad1.dpad_left;
            boolean right = gamepad1.dpad_right && !previousGamepad1.dpad_right;


            //gamepad 2
            boolean triangle2 = gamepad2.triangle && !previousGamepad2.triangle; //Turn off flywheel passive
            boolean square2 = gamepad2.square && !previousGamepad2.square; //Manual mode flywheel
            boolean circle2 = gamepad2.circle && !previousGamepad2.circle; //Relocalize limelight
            boolean X2 = gamepad2.cross && !previousGamepad2.cross; //Relocalize limelight
            boolean left2 = gamepad2.dpad_left && !previousGamepad2.dpad_left;
            boolean right2 = gamepad2.dpad_right && !previousGamepad2.dpad_right;
            boolean share2 = gamepad2.share && !previousGamepad1.share;



            boolean RB2 = gamepad2.right_bumper && !previousGamepad2.right_bumper; //Turn off turret
            boolean up2 = gamepad2.dpad_up && !previousGamepad2.dpad_up; //Increase flywheel manual power
            boolean down2 = gamepad2.dpad_down && !previousGamepad2.dpad_down; //Decrease flywheel manual power
            boolean options2 = gamepad2.options && !previousGamepad2.options;

            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);



            followerHandler.update();
            ShootingWhileMoving.update(followerHandler,lights.getTeamColor());
            flywheel.update();
            turret.update();
            hood.update();
            intake.update();
            gate.update();
            sorter.update();
            if (followerHandler.getState() == FollowerHandler.State.RESTING) {
                drivetrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            drivetrain.setYaw(followerHandler.getFollower().getHeading());
            lights.setBall(intake.getBallDetector().getBallColor());


            telemetry.addData("X", followerHandler.getFollower().getPose().getX());
            telemetry.addData("Y", followerHandler.getFollower().getPose().getY());
            telemetry.addData("Heading", followerHandler.getFollower().getPose().getHeading());

            lights.update(telemetry);
            webcam.update(telemetry);

            telemetry.update();

            webcam.updateFTCDashboard(telemetryPacket);
            flywheel.updateTelemetryPacket(telemetryPacket);
            followerHandler.ftcDashUpdate(telemetryPacket,lights.getTeamColor());
            turret.updateFTCDashboard(telemetryPacket);

            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        }
        lights.reset();
    }
}
