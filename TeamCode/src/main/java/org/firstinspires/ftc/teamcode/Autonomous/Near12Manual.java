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

    Pose startPose, shootPose, intakePoseFirst1, intakePoseFirst2, shootControlPose, gatePose, gateControlPose,shootControlPose2,intakePoseSecond1,intakePoseSecond2, intakePoseThird1, intakePoseThird2,leavePose;


    PathChain shootPrescore,
            intakeFirst1, intakeFirst2, shoot2, gateOpen, intakeSecond1, intakeSecond2,shoot3, shoot4, intakeThird1,intakeThird2, leave;

    public void buildPaths() {

        startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(130));

        intakePoseFirst1 = new Pose(calculateX(52.81543116490167), 57.45839636913767, calculateHeading(180));
        intakePoseFirst2 = new Pose(calculateX(11.043872919818456), 57.29349470499242, calculateHeading(180));

        gatePose = new Pose(calculateX(17.644478063540095),67.34795763993948,calculateHeading(180));
        gateControlPose = new Pose(calculateX(33.316944024205746),63.102874432677766);


        shootControlPose = new Pose(calculateX(54.56429652042359),64.996217851739765);

        shootControlPose2 = new Pose(calculateX(28.21988464447806),49.3018154311649);

        intakePoseSecond1 = new Pose(calculateX(44.75945537065052),84.52344931921331,calculateHeading(180));
        intakePoseSecond2 = new Pose(calculateX(18.844175491679277),84.7912254160363,calculateHeading(180));

        intakePoseThird1 = new Pose(calculateX(53.39183055975795),33.57186081694402,calculateHeading(180));
        intakePoseThird2 = new Pose(calculateX(13.502269288956139),33.534039334341905,calculateHeading(180));




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
        intakeFirst1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePoseFirst1
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(),calculateHeading(180))
                .build();
        intakeFirst2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePoseFirst1,
                                intakePoseFirst2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        gateOpen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakePoseFirst2,
                                gateControlPose,
                                gatePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(calculateHeading(180),shootPose.getHeading())
                .build();

        intakeSecond1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePoseSecond1
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(),calculateHeading(180))
                .build();
        intakeSecond2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePoseSecond1,
                                intakePoseSecond2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePoseSecond2,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(calculateHeading(180),shootPose.getHeading())
                .build();
        intakeThird1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakePoseThird1
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(),calculateHeading(180))
                .build();
        intakeThird2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePoseThird1,
                                intakePoseThird2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePoseThird2,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(calculateHeading(180),shootPose.getHeading())
                .build();


        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                leavePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(shootPose.getHeading())
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
            boolean dpadUp = gamepad1.dpad_up && !previousGamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down && !previousGamepad1.dpad_down;

            previousGamepad1.copy(gamepad1);

            if (options || triangle) {
                lights.switchTeamColor();
                buildPaths();
            }
            if (rb){
                buildPaths();
            }
            if (dpadUp){
                flywheel.add();
            }
            if (dpadUp){
                flywheel.sub();
            }
            flywheel.update();
            flywheel.status(telemetry);
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
           // turret.calculateOverrideAngle(lights.getTeamColor(),-45);
            turret.setState(Turret.States.RESET);
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
                    path += command.runShoot();
                    break;
                case 2:
                    path += command.runFollow(intakeFirst1,1200);
                    break;
                case 3:
                    path += command.runFollow(intakeFirst2,1200);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 4:
                    path += command.runFollow(gateOpen,1200);
                    intake.setState(Intake.States.OFF);
                    break;
                case 5:
                    path += command.runFollow(shoot2, 2000);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 6:
                    path += command.runShoot();
                    break;
                case 7:
                    path += command.runFollow(intakeSecond1,500);
                    break;
                case 8:
                    path += command.runFollow(intakeSecond2,1000);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 9:
                    intake.setState(Intake.States.OFF);
                    path += command.runFollow(shoot3, 1500);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 10:
                    path += command.runShoot();
                    break;
                case 11:
                    path += command.runFollow(intakeThird1,2500);
                    break;
                case 12:
                    path += command.runFollow(intakeThird2,1500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 13:
                    path += command.runFollow(shoot4, 3000);
                    flywheel.setState(Flywheel.States.SPINNING);
                    intake.setState(Intake.States.OFF);
                    break;
                case 14:
                    path += command.runShoot();
                    break;
                case 15:
                    path += command.runFollow(leave, 1000);
                    break;






            }

        }
    }
}
