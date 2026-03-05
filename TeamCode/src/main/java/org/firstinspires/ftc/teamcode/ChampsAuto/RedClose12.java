package org.firstinspires.ftc.teamcode.ChampsAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class RedClose12 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, timeoutTimer;
    private DcMotor intakeMotor = null;
    private DcMotor shootMotor = null;

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;
    private Servo door = null;
    private double GATE_DOWN_ANGLE = 0.30;
    private double GATE_UP_ANGLE = 0.60; //

    // state machine stuff
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;


    public enum PathState {
        //ROBOT STARTS AGAINST THE GOAL, CENTER OF THE ROBOT ON THE LINE
        DRIVE_STARTPOSE_SHOOTPOSE, SHOOTPRELOAD, //shoot preload
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE, STARTINTAKE1, DRIVE_LINEINTAKE1POSE_INTAKE1POSE, STOPINTAKE1,  //intake
        DRIVE_INTAKE1POSE_SHOOTPOSE, SHOOT1, DRIVE_SHOOTPOSE_LINEINTAKE2POSE, //shoot 1

        STARTINTAKE2, DRIVE_LINEINTAKE2POSE_INTAKE2POSE, STOPINTAKE2, //intake 2
        DRIVE_INTAKE2POSE_SHOOTPOSE, SHOOT2, //shoot 2

        DRIVE_SHOOTPOSE_LINEINTAKE3POSE, STARTINTAKE3, DRIVE_LINEINTAKE3POSE_INTAKE3POSE, STOPINTAKE3, //intake 3
        DRIVE_INTAKE3POSE_SHOOTPOSE2, SHOOT3, DRIVE_SHOOTPOSE2_LEAVEPOSE, //shoot 3
        DONE
    }


    PathState pathState;

    //all points
    private final Pose startPose = new Pose(145-33.4555712270804, 136.1579689703808, Math.toRadians(90));
    private final Pose shootPose = new Pose(145-55.85680170543313, 92.07533215512152, Math.toRadians(45));
    // shoot pose 2 in zone to get leave
    private final Pose shootPose2 = new Pose(145-59.6558533145275, 101.79266572637518, Math.toRadians(36));
    private final Pose lineIntake1Pose = new Pose(145-56.85680170543313, 87.07533215512152+4, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(145-16.08416494712284+5, 87.07533215512152+4, Math.toRadians(180));
    private final Pose lineIntake2Pose = new Pose(145-56.97981157469717, 63.524682651622+4, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(145-10.197060671580733+5, 63.524682651622+4, Math.toRadians(180));
    private final Pose lineIntake3Pose = new Pose(145-60.13540197461213, 34.54301833568405+3, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(145-9.545839210155147, 38.54301833568405+3, Math.toRadians(180));
    final Pose leavePose = new Pose(145-23.919605077574047, 93.42736248236953, Math.toRadians(90));

    //All the movement paths
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos,
            driveShootPosLineIntake2Pos, driveLineIntake2PosIntake2Pos, driveIntake2PosShootPos, driveShootPosLineIntake3Pos,
            driveLineIntake3PosIntake3Pos, driveIntake3PosShootPos2, driveShootPos2LeavePos;

    public void buildPaths() {
        //use coordinates of the points (above) for the starting position and the ending position to construct a path
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosLineIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineIntake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineIntake1Pose.getHeading())
                .build();
        driveLineIntake1PosIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(lineIntake1Pose.getHeading(), intake1Pose.getHeading())
                .build();
        driveIntake1PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosLineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2PosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosLineIntake3Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineIntake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineIntake3Pose.getHeading())
                .build();
        driveLineIntake3PosIntake3Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake3Pose, intake3Pose))
                .setLinearHeadingInterpolation(lineIntake3Pose.getHeading(), intake3Pose.getHeading())
                .build();
        driveIntake3PosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootPose2))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPos2LeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, leavePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), leavePose.getHeading())
                .build();
    }


    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        //any wait time in the multiconditional if statement takes place AFTER the path is run, and is the time that it takes for the entire path to run
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot preload");
                        setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                    }
                } break;


            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPosLineIntake1Pos, 0.95, true);
                    setPathState(PathState.STARTINTAKE1);
                }
                break;

            case STARTINTAKE1:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake first 3");
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSE);
                }
                break;

            case DRIVE_LINEINTAKE1POSE_INTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook 3 balls");
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.6, true);
                    setPathState(PathState.STOPINTAKE1);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy()) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE);
                }
                break;

            case DRIVE_INTAKE1POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy()){
                    //intakeMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE2POSE);
                    }
                } break;


            case DRIVE_SHOOTPOSE_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake2Pos, 0.8, true);
                    setPathState(PathState.STARTINTAKE2);
                }
                break;

            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSE);
                }
                break;


            case DRIVE_LINEINTAKE2POSE_INTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveLineIntake2PosIntake2Pos, 0.6, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;

            case STOPINTAKE2:
                if(!follower.isBusy()) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE);
                }
                break;


            case DRIVE_INTAKE2POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to preliminary shooting position 2 and rotated");
                    follower.followPath(driveIntake2PosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT2);
                }
                break;


            case SHOOT2:
                if(!follower.isBusy()){
                    //intakeMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot second 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE3POSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE_LINEINTAKE3POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake3Pos, 0.8, true);
                    setPathState(PathState.STARTINTAKE3);
                }
                break;

            case STARTINTAKE3:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake third 3");
                    setPathState(PathState.DRIVE_LINEINTAKE3POSE_INTAKE3POSE);
                }
                break;

            case DRIVE_LINEINTAKE3POSE_INTAKE3POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook third set of balls");
                    follower.followPath(driveLineIntake3PosIntake3Pos, 0.6, true);
                    setPathState(PathState.STOPINTAKE3);
                }
                break;

            case STOPINTAKE3:
                if(!follower.isBusy()) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked third 3");
                    setPathState(PathState.DRIVE_INTAKE3POSE_SHOOTPOSE2);
                }
                break;

            case DRIVE_INTAKE3POSE_SHOOTPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake3PosShootPos2, 0.9, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if(!follower.isBusy()){
                    //intakeMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot second 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE2_LEAVEPOSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE2_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Leave the zone");
                    follower.followPath(driveShootPos2LeavePos, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("DONE");
                break;

            default:
                telemetry.addLine("No state");
                break;
        }
    }


    public void setPathState (PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_SHOOTPOSE; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
        pathTimer = new Timer();
        opModeTimer = new Timer();
        timeoutTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        shooter.init(hardwareMap);
        shooter.TARGET_FLYWHEEL_RPM = shooter.CLOSE_FLYWHEEL_RPM;

        intakeMotor = hardwareMap.get(DcMotor.class, "i");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        shootMotor.setDirection(DcMotor.Direction.FORWARD);

        outtakeLeft = hardwareMap.get(DcMotorEx.class, "oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "oR");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);

        door = hardwareMap.get(Servo.class, "d");

        buildPaths();
        follower.setPose(startPose);
    }


    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    @Override
    public void loop(){
        follower.update();
        shooter.update();
        StatePathUpdate();

        telemetry.addData("Path State:", pathState.toString());
        //telemetry.addData("x:", follower.getPose().getX());
        //telemetry.addData("y:", follower.getPose().getY());
        //telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path time:", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Left flywheel velocity", outtakeLeft.getVelocity());
        telemetry.addData("Right flywheel velocity", outtakeRight.getVelocity());
        telemetry.addData("Shooting state", shooter.flywheelState);
        telemetry.addData("Shots triggered",shotsTriggered);
    }
}