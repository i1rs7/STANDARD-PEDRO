package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.Timer;

@TeleOp
public class AutoPathingPractice extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //ROBOT STARTS AGAINST THE GOAL, CENTER OF THE ROBOT ON THE LINE
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE,
        //SHOOT FIRST 3 BALLS, PRELOADED
        SHOOT_PRELOAD,
        //LINE UP TO INTAKE FIRST SET OF BALLS
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.12270803949224, 122.06488011283498, Math.toRadians(145));
    private final Pose shootPose = new Pose(55.44710860366714, 87.74047954866008, Math.toRadians(135));
    private final Pose lineIntake1Pose = new Pose(55.44710860366714, 87.74047954866008, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos;


    public void buildPaths(){
        //enter coordinates for starting position then ending positon using inputs above
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
            .build();
        driveShootPosLineIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineIntake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineIntake1Pose.getHeading())
                .build();
    }

    public void StatePathUpdate () {
        //update cases quickly
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //RESET TIMER & MAKE NEW PATH STATE
                telemetry.addLine("Moved back");
                        break;
            case SHOOT_PRELOAD:
                //check: has follower completed its path?
                //and check: has 3 seconds elapsed?
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3 ){
                    //TODO add flywheel logic??? He said to deleted it but idk
                    follower.followPath(driveShootPosLineIntake1Pos, true);
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                    telemetry.addLine("Shot 3???");
                }
                break;
            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake first set of balls");
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