package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Turret;

@Autonomous
public class Far extends LinearOpMode {
    private Follower follower;
    private Flywheel flywheel = new Flywheel();
    private Hood hood = new Hood();
    private Intake intake = new Intake();
    private Lights lights = new Lights();
    private Gate gate = new Gate();
    private Turret turret = new Turret();
    FollowerHandler followerHandler = new FollowerHandler();
    Command command;
    private int path = 0;

    private double calculateHeading(double heading) {
        return AutoPresets.calculateHeading(lights.getTeamColor(),heading);
    }
    private double calculateX(double x){
        return AutoPresets.calculateX(lights.getTeamColor(),x);
    }
    Pose startPose,shootPose,intakePose1, intakeControl1,intakePose2,leavePose;
    PathChain shootPrescore,
            intakeFirst, shoot2, intakeSecond, shoot3, leave;

    public void buildPaths() {

        startPose = new Pose(AutoPresets.calculateXStart(lights.getTeamColor(),56.653555219364605), 7.999999999999998, calculateHeading(180)); // Start Pose of our robot.

        intakePose1 = new Pose(calculateX(9.94402420574887), 35.7821482602118, calculateHeading(180));
        intakeControl1 = new Pose(calculateX(67.7836611195159), 38.79425113464447);

        shootPose = new Pose(calculateX(51.806354009077154), 13.978819969742814, calculateHeading(180));


        intakePose2 = new Pose(calculateX(14.263237518910739), 9.324508320726185, calculateHeading(180));


        leavePose = new Pose(calculateX(47.07715582450833), 27.61573373676248, calculateHeading(180));


        followerHandler.setStartingPose(startPose);

        shootPrescore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                intakeControl1,
                                intakePose1
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose1,
                                shootPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose2,
                                shootPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                leavePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        ShootingWhileMoving.update(followerHandler,lights.getTeamColor());
        turret.setState(Turret.States.AIM);
        turret.setState(Turret.States.AIM);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad previousGamepad1 = new Gamepad();
        followerHandler.initiate(hardwareMap);
        follower = followerHandler.getFollower();

        flywheel.initiate(hardwareMap);
        intake.initiate(hardwareMap);
        lights.initiate(hardwareMap);
        lights.setTeamColor(Lights.TeamColors.BLUE);
        gate.initiate(hardwareMap);
        turret.initiate(hardwareMap);
        turret.setState(Turret.States.AIM);
        hood.initiate(hardwareMap);
        hood.setState(Hood.States.ADJUST);

        command = new Command(flywheel, gate, intake, followerHandler);

        //Chamber
        buildPaths();
        boolean initial = true;
        while (!opModeIsActive() && !isStopRequested()) {
            if (initial) {
                turret.setState(Turret.States.RESET);
                initial = false;
            }
            boolean options = gamepad1.options && !previousGamepad1.options;
            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean rb = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            previousGamepad1.copy(gamepad1);

            if (options || triangle) {
                lights.switchTeamColor();
                buildPaths();
            }
            if (rb) {
                buildPaths();
            }

            lights.update(telemetry);
            telemetry.update();
            turret.update();

        }

        waitForStart();
        buildPaths();

        //   LimeLight limeLight = new LimeLight();
        //  limeLight.initiate(hardwareMap);
        if (isStopRequested()) {
            lights.save();
            return;
        }
        while (opModeIsActive()) {
            telemetry.addData("Auto path", path);

            followerHandler.update();
            flywheel.auto = true;
            ShootingWhileMoving.update(followerHandler,lights.getTeamColor());
            turret.setState(Turret.States.AIM);
            turret.calculateHeading();
            turret.update();
            hood.update();

            lights.save();
            followerHandler.saveLoop();


            flywheel.update();
            intake.update();
            lights.update(telemetry);
            lights.save();
            gate.update();


            flywheel.status(telemetry);
            gate.status(telemetry);


            telemetry.update();

            switch (path) {
                case 0:
                    turret.setState(Turret.States.MANUAL);
                    path += command.runFollow(shootPrescore,4000);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot(true);
                    break;
                case 2:
                    path += command.runFollow(intakeFirst, 1900);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 3:
                    path += command.runFollow(shoot2, 1900);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 4:
                    path += command.runShoot(true);
                    break;
                case 5:
                    path += command.runFollow(intakeSecond, 1800);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 6:
                    path += command.runFollow(shoot3, 1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 7:
                    path += command.runShoot(true);
                    break;
                case 8:
                    path += command.runFollow(intakeSecond, 1800);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 9:
                    path += command.runFollow(shoot3, 1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 10:
                    path += command.runShoot(true);
                    break;
                case 11:
                    path += command.runFollow(intakeSecond, 1800);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 12:
                    path += command.runFollow(shoot3, 1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 13:
                    path += command.runShoot(true);
                    break;
                case 14:
                    path += command.runFollow(intakeSecond, 1800);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 15:
                    path += command.runFollow(shoot3, 1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 16:
                    path += command.runShoot(true);
                    break;
                case 17:
                    path += command.runFollow(leave,1000);
                    break;


            }

        }
    }
}
