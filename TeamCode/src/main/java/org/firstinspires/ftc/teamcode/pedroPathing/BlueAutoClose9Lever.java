package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.FlywheelLogic;


@Autonomous
public class BlueAutoClose9Lever extends OpMode {


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //Flywheel logic
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotTriggered = false;


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
        DRIVE_INTAKE2POSE_LEVERPOSE,

        LEVER,

        DRIVE_LEVERPOSE_SHOOTPOSE,

        //Go to shooting position and shoot next 3

        SHOOT2,

        //Leave
        DRIVE_SHOOTPOSE_LEAVEPOSE,

        DONE
        //stop

    }










    PathState pathState;

    //all points
    private final Pose startPose = new Pose(20.919605077574047, 121.8617771509168, Math.toRadians(144));
    private final Pose shootPose = new Pose(64.856801705433135, 86.34008052590521, Math.toRadians(130));
    private final Pose lineIntake1Pose = new Pose(56.85680170543313, 84.07533215512152, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(21.08416494712284, 84.26914103802865, Math.toRadians(0));
    private final Pose lineIntake2Pose = new Pose(56.97981157469717, 58.524682651622, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(24.197060671580733, 58.524682651622, Math.toRadians(0));
    private final Pose leverPose = new Pose(16.24823695345557, 70.47672778561355, Math.toRadians(90));
    final Pose leavePose = new Pose(23.919605077574047, 93.42736248236953, Math.toRadians(90));












    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos,
            driveShootPosLineIntake2Pos, driveLineIntake2PosIntake2Pos, driveIntake2PosLeverPos, driveLeverPosShootPos, driveShootPosLeavePos;
















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
        driveIntake2PosLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, leverPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), leverPose.getHeading())
                .build();
        driveLeverPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, shootPose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), shootPose.getHeading())
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
                follower.followPath(driveStartPosShootPos, 0.9, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                // check if the path is done
                /*if (!follower.isBusy()){
                    //requested shots yet?
                    if (!shotTriggered) {
                        shooter.fireShots(3);
                        shotTriggered = true;
                    }
                    else if (shotTriggered && !shooter.isBusy()) {
                        follower.followPath(driveShootPosLineIntake1Pos, true);
                        setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                        telemetry.addLine("Shot preload");
                    }
                 }*/
                 if(!follower.isBusy()) {
                     telemetry.addLine("Shot preload");
                     setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE1POSE);
                 } break;


            case DRIVE_SHOOTPOSE_LINEINTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPosLineIntake1Pos, 0.8, true);
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
                if(!follower.isBusy()){
                    telemetry.addLine("Intook 3 balls");
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.6,true);
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
                    follower.followPath(driveIntake1PosShootPos, 0.9,true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy()) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot first 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LINEINTAKE2POSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPosLineIntake2Pos, 0.8,true);
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
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveLineIntake2PosIntake2Pos, 0.6, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;

            case STOPINTAKE2:
                if(!follower.isBusy()) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKE2POSE_LEVERPOSE);
                }
                break;


            case DRIVE_INTAKE2POSE_LEVERPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to preliminary shooting position 2 and rotated");
                    follower.followPath(driveIntake2PosLeverPos, 0.6, true);
                    setPathState(PathState.LEVER);
                }
                break;

            case LEVER:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4){
                    //TODO start flywheels
                    telemetry.addLine("Waiting at lever");
                    setPathState(PathState.DRIVE_LEVERPOSE_SHOOTPOSE);
                }
                break;


            case DRIVE_LEVERPOSE_SHOOTPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveLeverPosShootPos, 0.9, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if(!follower.isBusy()) {
                    //TODO add flywheel logic to shoot 3
                    telemetry.addLine("Shot second 3");
                    setPathState(PathState.DRIVE_SHOOTPOSE_LEAVEPOSE);
                }
                break;


            case DRIVE_SHOOTPOSE_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Leave the zone");
                    follower.followPath(driveShootPosLeavePos, true);
                }
                break;
            case DONE:
                telemetry.addLine("Done!");

            default:
                telemetry.addLine("No state");
                break;
        }
    }


























    public void setPathState (PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotTriggered = false;
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_SHOOTPOSE; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO ADD ANY OTHER INIT STUFF (FLYWHEEL, LIMELIGHT, ETC.)
        shooter.init(hardwareMap);
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
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path time:", pathTimer.getElapsedTimeSeconds());


    }
}