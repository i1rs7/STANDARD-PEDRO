package org.firstinspires.ftc.teamcode.ChampsAuto;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;






import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueClose12Lever extends OpMode {
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
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE1, SHOOT1, // shoot preload
        DRIVE_SHOOTPOSE1_LINEINTAKE1POSE, STARTINTAKE1, DRIVE_LINEINTAKE1POSE_INTAKE1POSE, STOPINTAKE1, // intake 1 (row 2)

        DRIVE_INTAKE1POSE_SHOOTPOSE2, SHOOT2,// shoot 1
        DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1, STARTINTAKE2,DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1, DRIVE_LEVERPOSE1_HITLEVERPOSE1, DRIVEHITLEVERPOSE1_LEVERPOSE1, STOPINTAKE2, // intake 2 (lever)
        DRIVE_LEVERPOSE1_SHOOTPOSE3, SHOOT3, // shoot 2
        DRIVE_SHOOTPOSE3_LINEINTAKE2, STARTINTAKE4, DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2, STOPINTAKE4,// intake 3 (row 1)
        DRIVE_INTAKEPOSE2_SHOOTPOSE4, SHOOT4, // shoot 3
        DONE //stop
    }

    PathState pathState;

    //all points
    private final Pose startPose = new Pose(33.4555712270804, 136.1579689703808, Math.toRadians(180-90));
    private final Pose shootPose1 = new Pose(53.85680170543313, 92.07533215512152, Math.toRadians(180-47));
    private final Pose shootPose2 = new Pose(53.856801705433135, 87.34008052590521, Math.toRadians(180-45));
    private final Pose shootPose3 = new Pose(55.856801705433135, 85.34008052590521, Math.toRadians(180-45));

    //make sure this on is inside the zone to get the leave points
    private final Pose shootPose4 = new Pose(55.6558533145275, 110.79266572637518, Math.toRadians(180-33));

    private final Pose lineIntake1Pose = new Pose(46.85680170543313, 66.524682651622-5, Math.toRadians(180-180));
    private final Pose intake1Pose = new Pose(12.08416494712284-10, 66.524682651622-7, Math.toRadians(180-180));
    private final Pose controlLever1 = new Pose (35.917366981341605, 68.60157710801516-7, Math.toRadians(180-220));
    private final Pose leverPose1 = new Pose(5, 68.60157710801516, Math.toRadians(180-220));
    private final Pose hitLeverPose = new Pose(5, 60.60157710801516, Math.toRadians(180-220));
    private final Pose lineIntake2Pose = new Pose(46.97981157469717, 89.07533215512152-3, Math.toRadians(180-180));
    private final Pose intake2Pose = new Pose(21.197060671580733-10, 89.07533215512152-3, Math.toRadians(180-180));

    //All the movement paths
    private PathChain driveStartPosShootPos, driveShootPos1LineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos2,
            driveShootPos2ControlLeverPose1, driveControlLeverPose1LeverPose1, driveLeverPose1HitLeverPos, driveHitLeverPosLeverPos1, driveLeverPose1ShootPose3, driveShootPos3LineIntake2,
            driveLineIntake2Intake2, driveIntake2ShootPose4;


    public void buildPaths(){
        //use coordinates of the points (above) for the starting position and the ending position to construct a path
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        driveShootPos1LineIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, lineIntake1Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), lineIntake1Pose.getHeading())
                .build();
        driveLineIntake1PosIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(lineIntake1Pose.getHeading(), intake1Pose.getHeading())
                .build();
        driveIntake1PosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shootPose2))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPos2ControlLeverPose1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, controlLever1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), controlLever1.getHeading())
                .build();
        driveControlLeverPose1LeverPose1 = follower.pathBuilder()
                .addPath(new BezierLine(controlLever1, leverPose1))
                .setLinearHeadingInterpolation(controlLever1.getHeading(), leverPose1.getHeading())
                .build();
        driveLeverPose1HitLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(leverPose1, hitLeverPose))
                .setLinearHeadingInterpolation(leverPose1.getHeading(), hitLeverPose.getHeading())
                .build();
        driveHitLeverPosLeverPos1 = follower.pathBuilder()
                .addPath(new BezierLine(hitLeverPose, leverPose1))
                .setLinearHeadingInterpolation(hitLeverPose.getHeading(), leverPose1.getHeading())
                .build();
        driveLeverPose1ShootPose3 = follower.pathBuilder()
                .addPath(new BezierLine(leverPose1, shootPose3))
                .setLinearHeadingInterpolation(leverPose1.getHeading(), shootPose3.getHeading())
                .build();
        driveShootPos3LineIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2ShootPose4 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose4))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose4.getHeading())
                .build();
    }

    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        //any wait time in the multiconditional if statement takes place AFTER the path is run, and is the time that it takes for the entire path to run
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE1:
                follower.followPath(driveStartPosShootPos, true); //Follow the path
                telemetry.addLine("Moved back");
                setPathState(PathState.SHOOT1); //RESET TIMER & SET TO NEXT PATH STATE
                break;

            case SHOOT1:
                telemetry.addLine("In SHOOT1");
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()> 0.2){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE1_LINEINTAKE1POSE);
                    }
                } break;

            case DRIVE_SHOOTPOSE1_LINEINTAKE1POSE:
                telemetry.addLine("In Drive_SP1_LI1P");
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPos1LineIntake1Pos, true);
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
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.65,true);
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
                    follower.followPath(driveIntake1PosShootPos2, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1);
                    }
                }
                break;

            case DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPos2ControlLeverPose1,true);
                    setPathState(PathState.STARTINTAKE2);
                }
                break;

            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1);
                }
                break;

            case DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveControlLeverPose1LeverPose1, 0.7, true);
                    setPathState(PathState.DRIVE_LEVERPOSE1_HITLEVERPOSE1);
                }
                break;

            case DRIVE_LEVERPOSE1_HITLEVERPOSE1:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveLeverPose1HitLeverPos, true);
                    setPathState(PathState.STOPINTAKE2);
                }

            case STOPINTAKE2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_LEVERPOSE1_SHOOTPOSE3);
                }
                break;

            case DRIVE_LEVERPOSE1_SHOOTPOSE3:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to preliminary lever pos ");
                    follower.followPath(driveLeverPose1ShootPose3, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE3_LINEINTAKE2);
                    }
                }
                break;

            case DRIVE_SHOOTPOSE3_LINEINTAKE2:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveShootPos3LineIntake2, true);
                    setPathState(PathState.STARTINTAKE4);
                }
                break;

            case STARTINTAKE4:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.75);
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2);
                }
                break;

            case DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveLineIntake2Intake2, 0.5, true);
                    setPathState(PathState.STOPINTAKE4);
                }
                break;

            case STOPINTAKE4:
                if(!follower.isBusy()) {
                    shootMotor.setPower(0);
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKEPOSE2_SHOOTPOSE4);
                }
                break;

            case DRIVE_INTAKEPOSE2_SHOOTPOSE4:
                if(!follower.isBusy()){
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveIntake2ShootPose4, true);
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DONE);
                    }
                }
                break;

            case DONE:
                if(!follower.isBusy())
                    telemetry.addLine("Done!");
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
        pathState = PathState.DRIVE_STARTPOSE_SHOOTPOSE1; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
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
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path time:", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("outtake left ticks",outtakeLeft.getVelocity());
        telemetry.addData("outtake right ticks",outtakeRight.getVelocity());




    }
}
