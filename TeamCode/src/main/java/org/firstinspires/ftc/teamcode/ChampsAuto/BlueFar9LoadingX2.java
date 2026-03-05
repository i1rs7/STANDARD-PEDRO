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

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogicFar;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BlueFar9LoadingX2 extends OpMode {

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
    private FlywheelLogicFar shooter = new FlywheelLogicFar();
    private boolean shotsTriggered = false;


    public enum PathState {
        DRIVE_STARTPOSE_SHOOTPOSE, //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        SHOOTPRELOAD,
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE, //LINE UP TO INTAKE FIRST SET OF BALLS
        STARTINTAKE1,
        DRIVE_LINEINTAKE1POSE_INTAKE1POSEx1,
        DRIVE_INTAKE1POSE_LINEINTAKE1POSEx1,
        DRIVE_LINEINTAKE1POSE_INTAKE1POSEx2,
        DRIVE_INTAKE1POSE_LINEINTAKE1POSEx2,
        DRIVE_LINEINTAKE1POSE_INTAKE1POSEx3,//Move back and intake first 3 balls + move balls down + start flywheels


        STOPINTAKE1,
        DRIVE_INTAKE1POSE_SHOOTPOSE2,//Return to shooting position, shoot
        SHOOT2,
        DRIVE_SHOOTPOSE2_LINEINTAKE2POSE,
        STARTINTAKE2,
        DRIVE_LINEINTAKE2POSE_INTAKE2POSEx1,
        DRIVE_INTAKE2POSE_LINEINTAKE2POSEx1,
        DRIVE_LINEINTAKE2POSE_INTAKE2POSEx2,
        DRIVE_INTAKE2POSE_LINEINTAKE2POSEx2,
        DRIVE_LINEINTAKE2POSE_INTAKE2POSEx3,
        STOPINTAKE2,
        DRIVE_INTAKE2POSE_SHOOTPOSE3,
        SHOOT3,
        DRIVE_SHOOTPOSE3_LEAVEPOSE,
        DONE //stop

    }

    PathState pathState;


    //all points
    private final Pose startPose = new Pose(56, 8, Math.toRadians(180-90));
    private final Pose shootPose1 = new Pose(56, 12, Math.toRadians(180-70+3));
    private final Pose shootPose2 = new Pose(56, 12, Math.toRadians(180-70+5));
    private final Pose shootPose3 = new Pose(56, 12, Math.toRadians(180-70+7));
    private final Pose lineIntake1Pose = new Pose(29.448183041722736, 8.77523553162853, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(9, 8.77523553162853, Math.toRadians(180));
    private final Pose lineIntake2Pose = new Pose(29.448183041722736, 8.77523553162853, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(9, 8.77523553162853, Math.toRadians(180));
    final Pose leavePose = new Pose(38, 15, Math.toRadians(180-180));



    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosLineIntake1Pos, driveIntake1PosShootPos2, driveShootPos2LineIntake2Pos, driveLineIntake2PosIntake2Pos, driveIntake2PosLineIntake2Pos,driveIntake2PosShootPos3, driveShootPos3LeavePos;


    public void buildPaths(){
        //use coordinates of the points (above) for the starting position and the ending position to construct a path
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        driveShootPosLineIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, lineIntake1Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), lineIntake1Pose.getHeading())
                .build();
        driveLineIntake1PosIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(lineIntake1Pose.getHeading(), intake1Pose.getHeading())
                .build();
        driveIntake1PosLineIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, lineIntake1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), lineIntake1Pose.getHeading())
                .build();
        driveIntake1PosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shootPose2))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPos2LineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2PosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2PosLineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, lineIntake2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveIntake2PosShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose3))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose3.getHeading())
                .build();
        driveShootPos3LeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, leavePose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), leavePose.getHeading())
                .build();
    }

    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        //any wait time in the multiconditional if statement takes place AFTER the path is run, and is the time that it takes for the entire path to run
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE:
                follower.followPath(driveStartPosShootPos, 0.95, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
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
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSEx1);
                }
                break;

            case DRIVE_LINEINTAKE1POSE_INTAKE1POSEx1:
                if(!follower.isBusy()){
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.DRIVE_INTAKE1POSE_LINEINTAKE1POSEx1);
                }
                break;

            case DRIVE_INTAKE1POSE_LINEINTAKE1POSEx1:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake1PosLineIntake1Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSEx2);
                }
                break;

            case DRIVE_LINEINTAKE1POSE_INTAKE1POSEx2:
                if(!follower.isBusy()){
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.DRIVE_INTAKE1POSE_LINEINTAKE1POSEx2);
                }
                break;

            case DRIVE_INTAKE1POSE_LINEINTAKE1POSEx2:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake1PosLineIntake1Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSEx3);
                }
                break;
            case DRIVE_LINEINTAKE1POSE_INTAKE1POSEx3:
                if(!follower.isBusy()){
                    follower.followPath(driveLineIntake1PosIntake1Pos, true);
                    setPathState(PathState.STOPINTAKE1);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    shootMotor.setPower(0);
                    intakeMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE2);
                }
                break;

            case DRIVE_INTAKE1POSE_SHOOTPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos2, 0.95,true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 1){

                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE2_LINEINTAKE2POSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE2_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPos2LineIntake2Pos, 0.95, true);
                    setPathState(PathState.STARTINTAKE2);
                }
                break;

            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake first 3");
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSEx1);
                }
                break;


            case DRIVE_LINEINTAKE2POSE_INTAKE2POSEx1:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake2PosLineIntake2Pos, true);
                    setPathState(PathState.DRIVE_INTAKE2POSE_LINEINTAKE2POSEx1);
                }
                break;

            case DRIVE_INTAKE2POSE_LINEINTAKE2POSEx1:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake2PosLineIntake2Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSEx2);
                }
                break;

            case DRIVE_LINEINTAKE2POSE_INTAKE2POSEx2:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake2PosLineIntake2Pos, true);
                    setPathState(PathState.DRIVE_INTAKE2POSE_LINEINTAKE2POSEx2);
                }
                break;

            case DRIVE_INTAKE2POSE_LINEINTAKE2POSEx2:
                if(!follower.isBusy()){
                    follower.followPath(driveIntake2PosLineIntake2Pos, true);
                    setPathState(PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSEx3);
                }
                break;


            case DRIVE_LINEINTAKE2POSE_INTAKE2POSEx3:
                if(!follower.isBusy()){
                    follower.followPath(driveLineIntake2PosIntake2Pos, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;

            case STOPINTAKE2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    shootMotor.setPower(0);
                    intakeMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE3);
                }
                break;

            case DRIVE_INTAKE2POSE_SHOOTPOSE3:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake2PosShootPos3, 0.95,true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 1){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE3_LEAVEPOSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE3_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveShootPos3LeavePos, 0.95,true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                if(!follower.isBusy()) {
                    outtakeLeft.setPower(0);
                    outtakeRight.setPower(0);
                    telemetry.addLine("Done!");
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
        shotsTriggered = false;
    }


    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOSE_SHOOTPOSE; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
        pathTimer = new Timer();
        opModeTimer = new Timer();
        timeoutTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        shooter.init(hardwareMap);
        //shooter.TARGET_FLYWHEEL_RPM = shooter.FAR_FLYWHEEL_RPM;

        intakeMotor = hardwareMap.get(DcMotor.class, "i");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        //  shooter.TARGET_FLYWHEEL_RPM = shooter.FA;

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
        outtakeLeft.setVelocity(shooter.FAR_FLYWHEEL_RPM);
        outtakeRight.setVelocity(shooter.FAR_FLYWHEEL_RPM);

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



