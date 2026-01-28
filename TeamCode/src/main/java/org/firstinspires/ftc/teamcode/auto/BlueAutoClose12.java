package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BlueAutoClose12 extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer, timeoutTimer;


    private DcMotor intakeMotor = null;
    private DcMotor shootMotor = null;

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;
    private Servo door = null;
    private double GATE_DOWN_ANGLE = 0.45;
    private double GATE_UP_ANGLE = 0.15; //


    // state machine stuff
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;


    public enum PathState {
        //ROBOT STARTS AGAINST THE GOAL, CENTER OF THE ROBOT ON THE LINE
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE,

        SHOOTPRELOAD,

        //LINE UP TO INTAKE FIRST SET OF BALLS
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE,

        STARTINTAKE1,

        //Move back and intake first 3 balls + move balls down + start flywheels
        DRIVE_LINEINTAKE1POSE_INTAKE1POSE,

        STOPINTAKE1,

        //Return to shooting position, shoot
        DRIVE_INTAKE1POSE_SHOOTPOSE,

        SHOOT1,

        //Line up to intake second set of three
        DRIVE_SHOOTPOSE_LINEINTAKE2POSE,

        STARTINTAKE2,

        //Intake second set of three + move balls down + start flywheels
        DRIVE_LINEINTAKE2POSE_INTAKE2POSE,

        STOPINTAKE2,

        //Go to first spot before shooting position to avoid the lever
        DRIVE_INTAKE2POSE_SHOOTPOSE,

        //Go to shooting position and shoot next 3

        SHOOT2,

        //Line up to intake next 3 balls
        DRIVE_SHOOTPOSE_LINEINTAKE3POSE,

        STARTINTAKE3,

        //Intake third set of three + move balls down + start flywheels
        DRIVE_LINEINTAKE3POSE_INTAKE3POSE,

        STOPINTAKE3,

        //Go to shooting position and shoot next 3
        DRIVE_INTAKE3POSE_SHOOTPOSE,

        SHOOT3,

        //Leave
        DRIVE_SHOOTPOSE_LEAVEPOSE,

        DONE
    }










    PathState pathState;

    //all points
    private final Pose startPose = new Pose(33.4555712270804, 136.1579689703808, Math.toRadians(90));
    private final Pose shootPose = new Pose(55.85680170543313, 92.07533215512152, Math.toRadians(130));
    private final Pose shootPose2 = new Pose(55.85680170543313, 92.07533215512152, Math.toRadians(135));
    private final Pose lineIntake1Pose = new Pose(56.85680170543313, 87.07533215512152, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(16.08416494712284, 87.07533215512152, Math.toRadians(0));
    private final Pose lineIntake2Pose = new Pose(56.97981157469717, 63.524682651622, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(15.197060671580733, 63.524682651622, Math.toRadians(0));
    private final Pose lineIntake3Pose = new Pose(60.13540197461213, 34.54301833568405, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(9.545839210155147, 38.54301833568405, Math.toRadians(0));
    final Pose leavePose = new Pose(23.919605077574047, 93.42736248236953, Math.toRadians(90));









    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos,
            driveShootPosLineIntake2Pos, driveLineIntake2PosIntake2Pos, driveIntake2PosShootPos, driveShootPosLineIntake3Pos,
            driveLineIntake3PosIntake3Pos, driveIntake3PosShootPos, driveShootPosLeavePos;
















    public void buildPaths(){
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
                .addPath(new BezierLine(intake1Pose, shootPose2))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPosLineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2PosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose2))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPosLineIntake3Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, lineIntake3Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), lineIntake3Pose.getHeading())
                .build();
        driveLineIntake3PosIntake3Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake3Pose, intake3Pose))
                .setLinearHeadingInterpolation(lineIntake3Pose.getHeading(), intake3Pose.getHeading())
                .build();
        driveIntake3PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootPose2))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPosLeavePos = follower.pathBuilder()
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
                outtakeLeft.setVelocity(770);
                outtakeRight.setVelocity(770);
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
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
                    follower.followPath(driveShootPosLineIntake1Pos, 0.8, true);
                    setPathState(PathState.STARTINTAKE1);
                }
                break;

            case STARTINTAKE1:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.95);
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
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE);
                }
                break;

            case DRIVE_INTAKE1POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
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
                    shootMotor.setPower(0.95);
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
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE);
                }
                break;


            case DRIVE_INTAKE2POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    telemetry.addLine("Moved to preliminary shooting position 2 and rotated");
                    follower.followPath(driveIntake2PosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT2);
                }
                break;


            case SHOOT2:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
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
                    shootMotor.setPower(0.95);
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
                    telemetry.addLine("Stopped intake after intaked third 3");
                    setPathState(PathState.DRIVE_INTAKE3POSE_SHOOTPOSE);
                }
                break;

            case DRIVE_INTAKE3POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake3PosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot second 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE_LEAVEPOSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Leave the zone");
                    follower.followPath(driveShootPosLeavePos, true);
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