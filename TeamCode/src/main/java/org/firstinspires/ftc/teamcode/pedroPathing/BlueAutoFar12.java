package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.pedropathing.util.Timer;


@Autonomous
public class BlueAutoFar12 extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        //ROBOT STARTS AGAINST THE GOAL, CENTER OF THE ROBOT ON THE LINE
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE,


        //LINE UP TO INTAKE FIRST SET OF BALLS
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE,


        //Move back and intake first 3 balls + move balls down + start flywheels
        DRIVE_LINEINTAKE1POSE_INTAKE1POSE,


        //Return to shooting position, shoot
        DRIVE_INTAKE1POSE_SHOOTPOSE,


        //Line up to intake second set of three
        DRIVE_SHOOTPOSE_LINEINTAKE2POSE,


        //Intake second set of three + move balls down + start flywheels
        DRIVE_LINEINTAKE2POSE_INTAKE2POSE,


        //Go to first spot before shooting position to avoid the lever
        DRIVE_INTAKE2POSE_SHOOTPOSE,


        //Line up to intake next 3 balls
        DRIVE_SHOOTPOSE_LINEINTAKE3POSE,


        //Intake third set of three + move balls down + start flywheels
        DRIVE_LINEINTAKE3POSE_INTAKE3POSE,

        //Go to shooting position and shoot next 3
        DRIVE_INTAKE3POSE_SHOOTPOSE,

        //Leave
        DRIVE_SHOOTPOSE_LEAVEPOSE,
    }










    PathState pathState;

    //all points
    private final Pose startPose = new Pose(56.05641748942172, 7.7179125528914, Math.toRadians(90));
    private final Pose shootPose = new Pose(64.78984485190409, 15.84203102961919, Math.toRadians(115));
    private final Pose lineIntake1Pose = new Pose(42.85472496473906, 35.54301833568405, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(7.717912552891397, 35.54301833568405, Math.toRadians(0));
    private final Pose lineIntake2Pose = new Pose(44.276445698166434, 60.7277856135402, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(20.716502115655853, 60.11847672778562, Math.toRadians(0));
    private final Pose lineIntake3Pose = new Pose(7.717912552891397, 35.74612129760225, Math.toRadians(90));
    private final Pose intake3Pose = new Pose(7.311706629055006, 7.108603667136817, Math.toRadians(90));
    private final Pose leavePose = new Pose(31.074753173483778, 23.35684062059238, Math.toRadians(115));












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
        driveIntake2PosShootPos= follower.pathBuilder()
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
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                //TODO start flywheels and shoot 3
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){ //do we need to wait three seconds if the shooting is in the if statement?
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPosLineIntake1Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSE);
                }
                break;


            case DRIVE_LINEINTAKE1POSE_INTAKE1POSE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()> 1){
                    //TODO start intake
                    telemetry.addLine("Intook 3 balls (point)");
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE);
                    //TODO intake pushes the balls down and that the flywheels start
                }
                break;


            case DRIVE_INTAKE1POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    //TODO add flywheel logic (intaking to push the balls up) (flywheels already started and balls are in place)
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos, true);
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


            case DRIVE_LINEINTAKE2POSE_INTAKE2POSE:
                if(!follower.isBusy()){
                    //TODO start intake
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveLineIntake2PosIntake2Pos, true);
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE);
                    //TODO intake pushes the balls down and that the flywheels start
                }
                break;


            case DRIVE_INTAKE2POSE_SHOOTPOSE:
                //TODO add flywheel logic (flywheels already started and balls are in place)
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake2PosShootPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE3POSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE3POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake3Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE3POSE_INTAKE3POSE);
                }
                break;


            case DRIVE_LINEINTAKE3POSE_INTAKE3POSE:
                if(!follower.isBusy()){
                    //TODO start intake
                    telemetry.addLine("Intook third set of balls");
                    follower.followPath(driveLineIntake3PosIntake3Pos, true);
                    setPathState(PathState.DRIVE_INTAKE3POSE_SHOOTPOSE);
                    //TODO intake pushes the balls down and that the flywheels start
                }
                break;


            case DRIVE_INTAKE3POSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    //TODO add flywheel logic (flywheels already started and balls are in place)
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake3PosShootPos, true);
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