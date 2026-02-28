package org.firstinspires.ftc.teamcode.NewSuperQualAuto;

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
public class BlueClose9 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, timeoutTimer;
    private DcMotor intakeMotor = null;
    private DcMotor shootMotor = null;
    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;
    private Servo door = null;
    private double GATE_DOWN_ANGLE = 0.15;
    private double GATE_UP_ANGLE = 0.45; //

    // state machine stuff
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;


    public enum PathState {
        DRIVE_STARTPOSE_SHOOTPOSE, //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        SHOOTPRELOAD,
        DRIVE_SHOOTPOSE_LINEINTAKE1POSE, //LINE UP TO INTAKE FIRST SET OF BALLS
        STARTINTAKE1,
        DRIVE_LINEINTAKE1POSE_INTAKE1POSE,//Move back and intake first 3 balls + move balls down + start flywheels
        STOPINTAKE1,
        DRIVE_INTAKE1POSE_SHOOTPOSE2,//Return to shooting position, shoot
        SHOOT1,
        DRIVE_SHOOTPOSE2_LINEINTAKE2POSE,
        STARTINTAKE2,
        DRIVE_LINEINTAKE2POSE_INTAKE2POSE, //Intake second set of three + move balls down + start flywheels
        STOPINTAKE2,
        DRIVE_INTAKE2POSE_SHOOTPOSE3,
        SHOOT2, //Go to shooting position and shoot next 3
        DRIVE_SHOOTPOSE3_LEAVEPOSE,  //Leave
        DONE //stop

    }



    PathState pathState;


    //all points
    private final Pose startPose = new Pose(33.4555712270804, 136.1579689703808, Math.toRadians(180-90));
    private final Pose shootPose = new Pose(55.85680170543313, 92.07533215512152, Math.toRadians(180-45));
    private final Pose shootPose2 = new Pose(53.856801705433135, 89.34008052590521, Math.toRadians(180-45));
    private final Pose shootPose3 = new Pose(50.856801705433135, 90.34008052590521, Math.toRadians(180-45));
    private final Pose lineIntake1Pose = new Pose(56.85680170543313, 87.07533215512152+4, Math.toRadians(180-180));
    private final Pose intake1Pose = new Pose(16.08416494712284+5, 87.07533215512152+4, Math.toRadians(180-180));
    private final Pose lineIntake2Pose = new Pose(56.97981157469717, 63.524682651622+4, Math.toRadians(180-180));
    private final Pose intake2Pose = new Pose(10.197060671580733+5, 63.524682651622+4, Math.toRadians(180-180));
    final Pose leavePose = new Pose(20.919605077574047, 93.42736248236953, Math.toRadians(180-90));



    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos, driveShootPosLineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos2,
            driveShootPos2LineIntake2Pos, driveLineIntake2PosIntake2Pos, driveIntake2PosShootPos3, driveShootPos3LeavePos;


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
                follower.followPath(driveStartPosShootPos, 0.9, true); //Follow the path
                setPathState(PathState.SHOOTPRELOAD); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

            case SHOOTPRELOAD:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(5);
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
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.6,true);
                    setPathState(PathState.STOPINTAKE1);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy()) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE2);
                }
                break;

            case DRIVE_INTAKE1POSE_SHOOTPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos2, 0.7,true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);


                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(5);
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
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPos2LineIntake2Pos, 0.95,true);
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
                    setPathState(PathState.DRIVE_INTAKE2POSE_SHOOTPOSE3);
                }
                break;

            case DRIVE_INTAKE2POSE_SHOOTPOSE3:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake2PosShootPos3, 0.8, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(5);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot second 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE3_LEAVEPOSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE3_LEAVEPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Leave the zone");
                    follower.followPath(driveShootPos3LeavePos, true);
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
        telemetry.addData("flywheel target rpm", shooter.TARGET_FLYWHEEL_RPM);
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



