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
public class Near12Manual extends LinearOpMode {
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
    private double calculateX(double x){
        if (lights.getTeamColor() == Lights.TeamColors.BLUE){
            return x;
        }
        return 144 - x;
    }

    Pose startPose, shootPose, intakePose1, intakeControl1, shootControlPose, gatePose, bounce,bounceControl,shootControlPose2,intakePose3,leavePose;


    PathChain shootPrescore,
            intakeFirst, shoot2, gateOpen, intakeSecond,shoot3, shoot4, intakeThird, leave;

    public void buildPaths() {

        startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(180));

        intakePose1 = new Pose(calculateX(14.662632375189096), 57.08169440242053, calculateHeading(180));
        intakeControl1 = new Pose(calculateX(54.778129727685325),50.70272314674733);


        shootControlPose = new Pose(calculateX(54.56429652042359),64.996217851739765);

        gatePose = new Pose(calculateX(16.036762481089247),62.40816944024204,calculateHeading(180));
        bounce = new Pose(calculateX(13.45990922844175),52.34039334341905,calculateHeading(167));
        bounceControl = new Pose(calculateX(26.936686838124047),49.18517397881995,calculateHeading(180));

        shootControlPose2 = new Pose(calculateX(28.21988464447806),49.3018154311649);

        intakePose3 = new Pose(calculateX(19.656580937972763),84.05295007564294,calculateHeading(180));

        leavePose = new Pose(calculateX(39.93948562783662),77.83358547655068,calculateHeading(180));






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
                                shootControlPose,
                                shootPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        gateOpen = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                gatePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gatePose,
                                bounceControl,
                                bounce
                        )
                )
                .setLinearHeadingInterpolation(calculateHeading(180),bounce.getHeading())
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gatePose,
                                shootControlPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(bounce.getHeading(),calculateHeading(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose3,
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
            lights.save();
            return;
        }
        while (opModeIsActive()) {
            telemetry.addData("Auto path", path);

            followerHandler.update();
            followerHandler.save();
            flywheel.auto = true;
            flywheel.calculateZone(shootPose,lights.getTeamColor());
            turret.setFollowerHandler(followerHandler);
            turret.calculateOverrideAngle(lights.getTeamColor(),-45);
            turret.setState(Turret.States.MANUAL);
            turret.setGoal(lights.getTeamColor());
            turret.update();

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
                    path += command.runFollow(shootPrescore, 2700);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot(true);
                    break;
                case 2:
                    path += command.runFollow(intakeFirst,1900);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 3:
                    path += command.runFollow(shoot2,1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 4:
                    path += command.runShoot(true);
                    break;
                case 5:
                    path += command.runFollow(gateOpen,1600);
                    break;
                case 6:
                    path += command.runFollow(intakeSecond,2200);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 7:
                    path += command.runFollow(shoot3,1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 8:
                    path += command.runShoot(true);
                    break;
                case 9:
                    path += command.runFollow(gateOpen,1600);
                    break;
                case 10:
                    path += command.runFollow(intakeSecond,2200);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 11:
                    path += command.runFollow(shoot3,1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 12:
                    path += command.runShoot(true);
                    break;
                case 13:
                    path += command.runFollow(intakeThird,1500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 14:
                    path += command.runFollow(shoot4,1500);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 15:
                    path += command.runShoot(true);
                    break;
                case 16:
                    path += command.runFollow(leave,1000);
                    break;



            }

        }
    }
}
