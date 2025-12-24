package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.pedropathing.util.Timer;

import org.opencv.core.Mat;


@Autonomous
public class BlueAutoComboAT extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        //ROBOT STARTS AGAINST THE GOAL, CENTER OF THE ROBOT ON THE LINE
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE,

        //Shoot preload
        SHOOTPRELOAD,

        //LINE UP TO INTAKE FIRST SET OF BALLS
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE,

        STARTINTAKE1,

        //Move back and intake first 3 balls + move balls down + start flywheels
        DRIVE_LINEINTAKE1POSE_INTAKE1POSE,

        STOPINTAKE1,

        //Move from intake position to line up with the lever
        DRIVE_INTAKE1POSE_LINELEVERPOSE,

        //Hit the lever
        DRIVE_LINELEVERPOSE_LEVERPOSE,

        HOLDLEVER,

        //Return to shooting position
        DRIVE_LEVERPOSE_SHOOTPOSE,

        SHOOT1,

        //Line up to intake next 3 balls
        DRIVE_SHOOTPOSE_LINEINTAKE2POSE,

        STARTINTAKE2,

        //Intake third set of three + move balls down + start flywheels
        DRIVE_LINEINTAKE2POSE_INTAKE2POSE,

        STOPINTAKE2

    }










    PathState pathState;

    //all points
    //TODO add intake to lever 1, lever 1 to lever 2, lever 2 to shoot
    private final Pose startPose = new Pose(20.919605077574047, 121.8617771509168, Math.toRadians(144));
    private final Pose shootPose = new Pose(48.13540197461213, 95.2552891396333, Math.toRadians(130));
    private final Pose lineIntake1Pose = new Pose(47.93229901269393, 83.88152327221438, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(14.82651622002821, 83.88152327221438, Math.toRadians(0));
    private final Pose lineLeverPose = new Pose(41.433004231311706, 75.75740479548661, Math.toRadians(90));
    private final Pose leverPose = new Pose(16.045133991537377, 74.53878702397743, Math.toRadians(90));
    private final Pose lineIntake2Pose = new Pose(48.33850493653032, 67.22708039492244, Math.toRadians(15));
    private final Pose intake2Pose = new Pose(8.32722143864598, 53.41607898448519, Math.toRadians(0));












    //All the movement paths, no intake/outtake here
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosLineLeverPos,
            driveLineLeverPosLeverPos, driveLeverPosShootPos, driveShootPosLineIntake2Pos, driveLineIntake2PosIntake2Pos;
















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
        driveIntake1PosLineLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, lineLeverPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), lineLeverPose.getHeading())
                .build();
        //TODO add intake to lever 1, lever 1 to lever 2, lever 2 to shoot
        driveLineLeverPosLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(lineLeverPose, leverPose))
                .setLinearHeadingInterpolation(lineLeverPose.getHeading(), leverPose.getHeading())
                .build();
        driveLeverPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, shootPose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosLineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2PosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
    }
















    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                //TODO start flywheels
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back, start flywheels");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5) {
                //TODO add flywheel logic to shoot 3
                telemetry.addLine("Shot 3");
                setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                }
                break;

            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()){ //do we need to wait three seconds if the shooting is in the if statement?
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPosLineIntake1Pos, true);
                    setPathState(PathState.STARTINTAKE1);
                }
                break;

            case STARTINTAKE1:
                if(!follower.isBusy()){
                    //TODO add intake logic to start intake
                    telemetry.addLine("Start intake");
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSE);
                }
                break;

            case DRIVE_LINEINTAKE1POSE_INTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook 3 balls (point)");
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.DRIVE_INTAKE1POSE_LINELEVERPOSE);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy()){
                    //TODO add intake logic to stop intake and push the balls down
                    telemetry.addLine("Stop intake");
                    setPathState(PathState.DRIVE_INTAKE1POSE_LINELEVERPOSE);
                }
                break;

            case DRIVE_INTAKE1POSE_LINELEVERPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Line up to hit the lever");
                    follower.followPath(driveIntake1PosLineLeverPos, true);
                    setPathState(PathState.DRIVE_LINELEVERPOSE_LEVERPOSE);
                }
                break;

            case DRIVE_LINELEVERPOSE_LEVERPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Hit the lever");
                    follower.followPath(driveLineLeverPosLeverPos, true);
                    setPathState(PathState.HOLDLEVER);
                }
                break;

            case HOLDLEVER:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4) {
                    //TODO start flywheels
                    telemetry.addLine("Waiting at lever");
                    setPathState(PathState.DRIVE_LEVERPOSE_SHOOTPOSE);
                }
                break;

            case DRIVE_LEVERPOSE_SHOOTPOSE:
                if(!follower.isBusy()) {
                    telemetry.addLine("Return to shooting position, shoot");
                    follower.followPath(driveLeverPosShootPos);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy()) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shoot 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE2POSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake2Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSE);
                }
                break;

            case STARTINTAKE2:
                if(!follower.isBusy()){
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake");
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSE);
                }
                break;

            case DRIVE_LINEINTAKE2POSE_INTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls, leave");
                    follower.followPath(driveLineIntake2PosIntake2Pos, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;

            case STOPINTAKE2:
                if(!follower.isBusy()){
                    //TODO add intake logic to stop intake
                    telemetry.addLine("Stopped intake");
                }
                break;







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



