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
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous
public class Near15Ball extends LinearOpMode {
    private Follower follower;
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Lights lights = new Lights();
    private Gate gate = new Gate();
    private Turret turret = new Turret();
    FollowerHandler followerHandler = new FollowerHandler();
    Command command;
    private int path = 0;

    private double calculateHeading(double heading) {
        if (lights.getTeamColor() == Lights.TeamColors.BLUE) {
            return Math.toRadians(heading);
        }
        return Math.toRadians(180 - heading);
    }

    private double calculateX(double x) {
        if (lights.getTeamColor() == Lights.TeamColors.BLUE) {
            return x;
        }
        return 144 - x;
    }
    Pose startPose,shootPose,intakePose1, intakeControl1,shootPose2,shootControl2,intakePose2,shootPose3,intakePose3, intakeControl3,intakePose4,intakeControl4,leavePose;
    PathChain shootPrescore,
            intakeFirst, shoot2, intakeSecond, shoot3, intakeThird, shoot4, intakeFourth, shoot5, leave;

    public void buildPaths() {

        startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(180));

        intakePose1 = new Pose(calculateX(14.662632375189096), 57.08169440242053, calculateHeading(180));
        intakeControl1 = new Pose(calculateX(54.778129727685325), 50.70272314674733);


        shootPose2 = new Pose(calculateX(50.627836611195164), 88.95310136157337, calculateHeading(180));
        shootControl2 = new Pose(calculateX(54.56429652042359), 58.996217851739765);

        intakePose2 = new Pose(calculateX(18.95915279878971), 73.65779122541604, calculateHeading(180));

        shootPose3 = new Pose(calculateX(57.11195158850227), 73.55673222390317, calculateHeading(180));


        intakePose3 = new Pose(calculateX(18.461422087745841), 36.236006051437215, calculateHeading(180));
        intakeControl3 = new Pose(calculateX(64.728441754916794), 25.568078668683814);

        intakePose4 = new Pose(calculateX(12.386384266263238), 7.01694402420574, calculateHeading(180));
        intakeControl4 = new Pose(calculateX(46.30786686838125), 29.584720121028724);

        leavePose = new Pose(calculateX(32.161875945537055), 66.82904689863841, calculateHeading(180));


        followerHandler.setStartingPose(startPose);
        followerHandler.save();

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
                        new BezierCurve(
                                intakePose1,
                                shootControl2,
                                shootPose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose2,
                                intakePose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose2,
                                shootPose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose3,
                                intakeControl3,
                                intakePose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose3,
                                shootPose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        intakeFourth = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose3,
                                intakeControl4,
                                intakePose4
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose4,
                                shootPose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose3,
                                leavePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

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

        command = new Command(flywheel, gate, intake, followerHandler);

        //Chamber
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
            followerHandler.save();
            flywheel.auto = true;
            turret.setFollowerHandler(followerHandler);
            turret.setGoal(lights.getTeamColor());
            turret.setState(Turret.States.AIM);
            turret.update();


            flywheel.calculateZone(follower.getPose(), lights.getTeamColor());


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
                    path += command.runFollow(shootPrescore, 2900);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot();
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
                    path += command.runShoot();
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
                    path += command.runShoot();
                    break;
                case 8:
                    path += command.runFollow(intakeThird, 2700);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 9:
                    path += command.runFollow(shoot4, 2100);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 10:
                    path += command.runShoot();
                    break;
                case 11:
                    path += command.runFollow(intakeFourth, 3500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 12:
                    path += command.runFollow(shoot5, 2500);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 13:
                    path += command.runShoot();
                    break;
                case 14:
                    path += command.runFollow(leave,1000);
                    break;


            }

        }
    }
}
