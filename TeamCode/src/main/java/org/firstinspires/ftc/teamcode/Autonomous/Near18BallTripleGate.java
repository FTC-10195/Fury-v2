package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.Stopwatch;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;

@Autonomous
@Config
public class Near18BallTripleGate extends LinearOpMode {
    private Follower follower;
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Lights lights = new Lights();
    private Gate gate = new Gate();
    private Turret turret = new Turret();
    private Hood hood = new Hood();
    FollowerHandler followerHandler = new FollowerHandler();
    Command command;
    Stopwatch stopwatch = new Stopwatch();
    private int path = 0;
    public static double preGateX = 28.869686985172972;
    public static double preGateY = 60.87775947281715;
    public static double gateControlX = 41.51345898020323;
    public static double gateControlY = 60.77177109217477;
    public static long gateTime = 3000;
    public static long preGateTime = 800;
    public static double gateX = 12.2321252059308076;
    public static double gateY = 59.705107084019765;
    public static double gateHeading = 155;
    public static double redGateOffsetX = 0;//4; //THIS SHOULD NOT EXIST IDK WHAT IS WRONG WITH RED
    private double calculateHeading(double heading) {
        return AutoPresets.calculateHeading(lights.getTeamColor(),heading);
    }
    private double calculateX(double x){
        return AutoPresets.calculateX(lights.getTeamColor(),x);
    }
    private double gateCalculate(double x){
        if (lights.getTeamColor() == Lights.TeamColors.BLUE){
            return x;
        }
        return calculateX(x) - redGateOffsetX;
    }

    Pose startPose, shootPose, intakeFirstPose, intakeFirstControl1,intakeFirstControl2, shootControlPose, gatePose, intakeSecondPose, gateControl,gateControl2, shootControlPoseThird1,shootControlPoseThird2,intakePose3,intakePose4,leavePose;

    PathChain shootPrescore,
            intakeFirst, shoot2, gateOpen, intakeSecond,shoot3, shoot4, intakeThird,intakeFourth,shoot5, leave;

    public void buildPaths() {

        //Linear heading interpolation
        startPose = new Pose(AutoPresets.calculateXStart(lights.getTeamColor(),32.27677100494234), 133.02141680395385, calculateHeading(270)); // Start Pose of our robot.
        shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(225));

        //Tangential
        intakeFirstPose = new Pose(calculateX(12.00494233937396), 56.67380560131796);
        intakeFirstControl1 = new Pose(calculateX(35.460461285008236),62.4011532125206);
        intakeFirstControl2 = new Pose(calculateX(21.12438220757825),55.26606260296541);

        //Shoot Tangential Reverse



        shootControlPose = new Pose(calculateX(37.63957100656236),66.88890453533784);

        //Tangential
        gatePose = new Pose(gateCalculate(preGateX), preGateY,calculateHeading(155));
        gateControl = new Pose(gateCalculate(gateControlX),gateControlY);
     //   gateControl2 = new Pose(gateCalculate(21.190724219082956),58.85743830799024);

        //Linear
        intakeSecondPose = new Pose(gateCalculate(gateX),gateY,calculateHeading(gateHeading));

        //Tangential Reverse
        shootControlPoseThird1 = new Pose(calculateX(35.412223395359725),58.07012364571677);
       // shootControlPoseThird1 = new Pose(calculateX(42.47812125367933),75.8806673030479);

        //Tanget
        intakePose3 = new Pose(calculateX(14.841845140032952),83.500823723229,calculateHeading(180));
        //Shoot Linear

        //Intake Constant
        intakePose4 = new Pose(calculateX(24.621087314662294),29.09225700164748,calculateHeading(225));

        //Leave constant
        leavePose = new Pose(calculateX(49.8171334431631),111.53377265238879,calculateHeading(225));






        followerHandler.setStartingPose(startPose);

        shootPrescore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                intakeFirstControl1,
                                intakeFirstControl2,
                                intakeFirstPose
                        )
                )
                .setGlobalTangentHeadingInterpolation()
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeFirstPose,
                                shootControlPose,
                                shootPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateOpen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                gateControl,
                                gatePose
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                intakeSecondPose
                        )
                )
                .setConstantHeadingInterpolation(intakeSecondPose.getHeading())
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeSecondPose,
                                shootControlPoseThird1,
                                shootPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePose3
                        )
                )
                .setGlobalTangentHeadingInterpolation()
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose3,
                                leavePose
                        )
                )
                .setGlobalTangentHeadingInterpolation()
                .setGlobalReversed()
                .build();
        intakeFourth = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePose4
                        )
                )
                .setGlobalConstantHeadingInterpolation(intakePose4.getHeading())
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose4,
                                leavePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(leavePose.getHeading())
                .build();
        ShootingWhileMoving.update(followerHandler,lights.getTeamColor());
        turret.setState(Turret.States.AIM);
    }

    int prevPath = 0;

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
        hood.initiate(hardwareMap);
        turret.initiate(hardwareMap);
        turret.setState(Turret.States.AIM);

        buildPaths();
        command = new Command(flywheel, gate, intake, followerHandler);

        //Chamber
        boolean initial = true;
        while (!opModeIsActive() && !isStopRequested()) {
            if (initial){
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
            if (rb){
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
            return;
        }
        while (opModeIsActive()) {
            telemetry.addData("Auto path", path);

            followerHandler.update();
            flywheel.auto = true;
            ShootingWhileMoving.update(followerHandler,lights.getTeamColor());
            turret.setState(Turret.States.AIM);
            turret.setState(Turret.States.AIM);
            turret.calculateHeading();
            turret.update();
            flywheel.setState(Flywheel.States.SPINNING);

            flywheel.update();
            intake.update();
            lights.update(telemetry);
            lights.save();
            followerHandler.saveLoop();
            gate.update();
            hood.update();


            flywheel.status(telemetry);
            gate.status(telemetry);


            telemetry.update();

            if (path != prevPath){
                stopwatch.reset();
            }
            prevPath = path;
            stopwatch.run();

            switch (path) {
                case 0:
                    path += command.runFollow(shootPrescore, 2300);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot(true);
                    break;
                case 2:
                    path += command.runFollow(intakeFirst, 1400);
                    if (stopwatch.getTimePassed() > 500) {
                        intake.setState(Intake.States.INTAKE);
                    }

                    break;
                case 3:
                    turret.calculateOverrideAngle(lights.getTeamColor(), -100);
                    path += command.runFollow(shoot2, 1400);
                    if (stopwatch.getTimePassed() > 800) {
                        intake.setState(Intake.States.OFF);
                    }
                    flywheel.setState(Flywheel.States.SPINNING);


                    break;
                case 4:
                    path += command.runShoot(true);
                    break;
                case 5:
                    path += command.runFollow(gateOpen, preGateTime);
                    break;
                case 6:
                    path += command.runFollow(intakeSecond, gateTime);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 7:
                    path += command.runFollow(shoot3, 1400);
                    flywheel.setState(Flywheel.States.SPINNING);
                    if (stopwatch.getTimePassed() > 500) {
                        intake.setState(Intake.States.OFF);
                    } else {
                        intake.setState(Intake.States.INTAKE);
                    }
                    break;
                case 8:
                    path += command.runShoot(true);
                    break;
                case 9:
                    path += command.runFollow(gateOpen, preGateTime);
                    break;
                case 10:
                    path += command.runFollow(intakeSecond, gateTime);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 11:
                    path += command.runFollow(shoot3, 1400);
                    flywheel.setState(Flywheel.States.SPINNING);
                    if (stopwatch.getTimePassed() > 500) {
                        intake.setState(Intake.States.OFF);
                    } else {
                        intake.setState(Intake.States.INTAKE);
                    }
                    break;
                case 12:
                    path += command.runShoot(true);
                    break;
                case 13:
                    path += command.runFollow(gateOpen, preGateTime - 200);
                    break;
                case 14:
                    path += command.runFollow(intakeSecond, gateTime);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 15:
                    path += command.runFollow(shoot3, 1400);
                    flywheel.setState(Flywheel.States.SPINNING);
                    if (stopwatch.getTimePassed() > 300) {
                        intake.setState(Intake.States.OFF);
                    } else {
                        intake.setState(Intake.States.INTAKE);
                    }
                    break;
                case 16:
                    path += command.runShoot(true);
                    break;
                case 17:
                    path += command.runFollow(intakeThird, 1000);
                    if (stopwatch.getTimePassed() > 300) {
                        intake.setState(Intake.States.INTAKE);
                    } else {
                        intake.setState(Intake.States.OFF);
                    }                    break;
                case 18:
                    path += command.runFollow(shoot4, 1200);
                    flywheel.setState(Flywheel.States.SPINNING);
                    if (stopwatch.getTimePassed() > 300) {
                        intake.setState(Intake.States.OFF);
                    } else {
                        intake.setState(Intake.States.INTAKE);
                    }
                    break;
                case 19:
                    path += command.runShoot(true);
                    break;

            }

        }
    }
}
