package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.pedropathing.util.Timer;


@Autonomous
public class RedAutoFar12 extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer;


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
    }










    PathState pathState;

    //all points
    private final Pose startPose = new Pose(86.52186177715092, 8.733427362482372, Math.toRadians(90));
    private final Pose shootPose = new Pose(79.21015514809591, 15.84203102961919, Math.toRadians(67));
    private final Pose lineIntake1Pose = new Pose(101.14527503526094, 35.54301833568405, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(134.2820874471086, 35.54301833568405, Math.toRadians(180));
    private final Pose lineIntake2Pose = new Pose(99.72355430183357, 60.7277856135402, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(123.28349788434415, 60.11847672778562, Math.toRadians(180));
    private final Pose lineIntake3Pose = new Pose(134.2820874471086, 35.74612129760225, Math.toRadians(90));
    private final Pose intake3Pose = new Pose(134.688293370945, 9.54583921015515, Math.toRadians(90));
    private final Pose leavePose = new Pose(112.92524682651622, 23.35684062059238, Math.toRadians(67));












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
        driveIntake3PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();
    }
















    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        //any wait time in the multiconditional if statement takes place AFTER the path is run, and is the time that it takes for the entire path to run
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                //TODO start flywheels
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot preload");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPosLineIntake1Pos, true);
                    setPathState(PathState.STARTINTAKE1);
                }
                break;

            case STARTINTAKE1:
                if(!follower.isBusy()) {
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake first 3");
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSE);
                }
                break;

            case DRIVE_LINEINTAKE1POSE_INTAKE1POSE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    telemetry.addLine("Intook 3 balls");
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.STOPINTAKE1);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy()) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE);
                }
                break;

            case DRIVE_INTAKE1POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot first 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE2POSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake2Pos, true);
                    setPathState(PathState.STARTINTAKE2);
                }
                break;

            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSE);
                }
                break;


            case DRIVE_LINEINTAKE2POSE_INTAKE2POSE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveLineIntake2PosIntake2Pos, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;

            case STOPINTAKE2:
                if(!follower.isBusy()) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE);
                }
                break;


            case DRIVE_INTAKE2POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to preliminary shooting position 2 and rotated");
                    follower.followPath(driveIntake2PosShootPos, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot second 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE3POSE);
                }
                break;

            case DRIVE_SHOOTPOSE_LINEINTAKE3POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake3Pos, true);
                    setPathState(PathState.STARTINTAKE3);
                }
                break;

            case STARTINTAKE3:
                if(!follower.isBusy()) {
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake third 3");
                    setPathState(PathState.DRIVE_LINEINTAKE3POSE_INTAKE3POSE);
                }
                break;

            case DRIVE_LINEINTAKE3POSE_INTAKE3POSE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    telemetry.addLine("Intook third set of balls");
                    follower.followPath(driveLineIntake3PosIntake3Pos, true);
                    setPathState(PathState.STOPINTAKE3);
                }
                break;

            case STOPINTAKE3:
                if(!follower.isBusy()) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked third 3");
                    setPathState(PathState.DRIVE_INTAKE3POSE_SHOOTPOSE);
                }
                break;

            case DRIVE_INTAKE3POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake3PosShootPos, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot third 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LEAVEPOSE);
                }
                break;

            case DRIVE_SHOOTPOSE_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Leave the zone - DONE!!!");
                    follower.followPath(driveShootPosLeavePos, true);
                }










            default:
                telemetry.addLine("No state");
                break;
        }
    }


























    public void setPathState (PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_SHOOTPOSE; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO ADD ANY OTHER INIT STUFF (FLYWHEEL, LIMELIGHT, ETC.)
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
        StatePathUpdate();


        telemetry.addData("Path State:", pathState.toString());
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path time:", pathTimer.getElapsedTimeSeconds());


    }
}