package org.firstinspires.ftc.teamcode.SuperQualAuto;


//import static java.lang.Thread.sleep;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;






import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//import org.firstinspires.ftc.teamcode.pedroPathing.FlywheelLogic;




@Autonomous
public class BlueCloseAuto extends OpMode {

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

    //Flywheel logic
    //private FlywheelLogic shooter = new FlywheelLogic();

    public enum PathState {
        //MOVE BACK & ROTATE 5 DEGREES TO FACE GOAL
        DRIVE_STARTPOSE_SHOOTPOSE1,


        SHOOT1,


        //LINE UP TO INTAKE FIRST SET OF BALLS
        DRIVE_SHOOTPOSE1_LINEINTAKE1POSE,


        STARTINTAKE1,


        //Move back and intake first 3 balls + move balls down + start flywheels
        DRIVE_LINEINTAKE1POSE_INTAKE1POSE,


        STOPINTAKE1,


        //Return to shooting position, shoot
        DRIVE_INTAKE1POSE_SHOOTPOSE2,


        SHOOT2,


        //Line up to intake second set of three
        DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1,


        STARTINTAKE2,

        DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1,


        STOPINTAKE2,

        DRIVE_LEVERPOSE1_SHOOTPOSE3,

        SHOOT3,

        DRIVE_SHOOTPOSE3_CONTROLLEVERPOSE2,

        STARTINTAKE3,

        DRIVE_CONTROLLEVERPOSE2_LEVERPOSE2,

        STOPINTAKE3,

        DRIVE_LEVERPOSE2_SHOOTPOSE4,

        SHOOT4,

        DRIVE_SHOOTPOSE4_LINEINTAKE2,

        STARTINTAKE4,

        //THIS POSE IS INSIDE THE ZONE TO GET LEAVE POINTS
        DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2,

        STOPINTAKE4,

        DRIVE_INTAKEPOSE2_SHOOTPOSE5,

        SHOOT5,

        DONE
        //stop


    }














    PathState pathState;


    //all points
    private final Pose startPose = new Pose(33.4555712270804, 136.1579689703808, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(55.85680170543313, 92.07533215512152, Math.toRadians(135));
    private final Pose shootPose2 = new Pose(53.856801705433135, 89.34008052590521, Math.toRadians(145));
    private final Pose shootPose3 = new Pose(50.856801705433135, 90.34008052590521, Math.toRadians(135));
    private final Pose shootPose4 = new Pose(48, 90, Math.toRadians(135));

    //make sure this on is inside the zone to get the leave points
    private final Pose shootPose5 = new Pose(46, 90, Math.toRadians(135));



    private final Pose lineIntake1Pose = new Pose(56.97981157469717, 60.524682651622, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(24.197060671580733, 60.524682651622, Math.toRadians(0));
    private final Pose controlLever1 = new Pose (35.917366981341605, 69.60157710801516, Math.toRadians(90));
    private final Pose leverPose1 = new Pose(16.54823695345557, 67.47672778561355, Math.toRadians(90));
    private final Pose controlLever2 = new Pose (35.917366981341605, 69.60157710801516, Math.toRadians(90));
    private final Pose leverPose2 = new Pose(16.54823695345557, 67.47672778561355, Math.toRadians(90));
    private final Pose lineIntake2Pose = new Pose(56.85680170543313, 84.07533215512152, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(21.08416494712284, 84.07533215512152, Math.toRadians(0));
















    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos, driveShootPos1LineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos2,
            driveShootPos2ControlLeverPose1, driveControlLeverPose1LeverPose1, driveLeverPose1ShootPose3, driveShootPos3ControlLeverPose2, driveControlLeverPose2LeverPose2, driveLeverPose2ShootPose4, driveShootPose4LineIntake2, driveLineIntake2Intake2, driveIntake2ShootPose5;



















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
        driveLeverPose1ShootPose3 = follower.pathBuilder()
                .addPath(new BezierLine(leverPose1, shootPose3))
                .setLinearHeadingInterpolation(leverPose1.getHeading(), shootPose3.getHeading())
                .build();
        driveShootPos3ControlLeverPose2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, controlLever2))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), controlLever2.getHeading())
                .build();
        driveControlLeverPose2LeverPose2 = follower.pathBuilder()
                .addPath(new BezierLine(controlLever2, leverPose2))
                .setLinearHeadingInterpolation(controlLever2.getHeading(), leverPose2.getHeading())
                .build();
        driveLeverPose2ShootPose4 = follower.pathBuilder()
                .addPath(new BezierLine(leverPose2, shootPose4))
                .setLinearHeadingInterpolation(leverPose2.getHeading(), shootPose4.getHeading())
                .build();
        driveShootPose4LineIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose4, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2ShootPose5 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose5))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose5.getHeading())
                .build();
    }














    public void StatePathUpdate () {
        //update cases quickly
        //run the paths in order
        //any wait time in the multiconditional if statement takes place AFTER the path is run, and is the time that it takes for the entire path to run
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOTPOSE1:
                outtakeLeft.setVelocity(770);
                outtakeRight.setVelocity(770);
                follower.followPath(driveStartPosShootPos, 0.9, true); //Follow the path
                setPathState(PathState.SHOOT1); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
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
                        setPathState(PathState.DRIVE_SHOOTPOSE1_LINEINTAKE1POSE);
                    }
                } break;




            case DRIVE_SHOOTPOSE1_LINEINTAKE1POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPos1LineIntake1Pos, 0.8, true);
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
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.6,true);
                    setPathState(PathState.STOPINTAKE1);
                }
                break;


            case STOPINTAKE1:
                if(!follower.isBusy()) {
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(PathState.DRIVE_INTAKE1POSE_SHOOTPOSE2);
                }
                break;


            case DRIVE_INTAKE1POSE_SHOOTPOSE2:
                intakeMotor.setPower(0);
                shootMotor.setPower(0);
                door.setPosition(GATE_DOWN_ANGLE);
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos2, 0.7,true);
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
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1);
                    }
                }
                break;


            case DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPos2ControlLeverPose1, 0.8,true);
                    setPathState(PathState.STARTINTAKE2);
                }
                break;


            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.95);
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1);
                }
                break;


            case DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveControlLeverPose1LeverPose1, 0.6, true);
                    setPathState(PathState.STOPINTAKE2);
                }
                break;


            case STOPINTAKE2:
                if(!follower.isBusy()) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_LEVERPOSE1_SHOOTPOSE3);
                }
                break;



            case DRIVE_LEVERPOSE1_SHOOTPOSE3:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    door.setPosition(GATE_DOWN_ANGLE);
                    telemetry.addLine("Moved to preliminary lever pos ");
                    follower.followPath(driveLeverPose1ShootPose3, 0.6, true);
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
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE3_CONTROLLEVERPOSE2);
                    }
                }
                break;

            case DRIVE_SHOOTPOSE3_CONTROLLEVERPOSE2:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveShootPos3ControlLeverPose2, 0.6, true);
                    setPathState(PathState.STARTINTAKE3);
                }
                break;

            case STARTINTAKE3:
                if(!follower.isBusy()) {
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_CONTROLLEVERPOSE2_LEVERPOSE2);
                }
                break;

            case DRIVE_CONTROLLEVERPOSE2_LEVERPOSE2:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.95);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveControlLeverPose2LeverPose2, 0.6, true);
                    setPathState(PathState.STARTINTAKE3);
                }
                break;

            case STOPINTAKE3:
                if(!follower.isBusy()) {
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_LEVERPOSE2_SHOOTPOSE4);
                }
                break;

            case DRIVE_LEVERPOSE2_SHOOTPOSE4:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveLeverPose2ShootPose4, 0.6, true);
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(PathState.DRIVE_SHOOTPOSE4_LINEINTAKE2);
                    }
                }
                break;

            case DRIVE_SHOOTPOSE4_LINEINTAKE2:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveShootPose4LineIntake2, 0.6, true);
                    setPathState(PathState.STARTINTAKE4);
                }
                break;

            case STARTINTAKE4:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.95);
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(PathState.DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2);
                }
                break;

            case DRIVE_LINEINTAKEPOSE2_INTAKEPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveLineIntake2Intake2, 0.6, true);
                    setPathState(PathState.STOPINTAKE4);
                }
                break;

            case STOPINTAKE4:
                if(!follower.isBusy()) {
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(PathState.DRIVE_INTAKEPOSE2_SHOOTPOSE5);
                }
                break;

            case DRIVE_INTAKEPOSE2_SHOOTPOSE5:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    telemetry.addLine("Lever pose and rotated");
                    follower.followPath(driveIntake2ShootPose5, 0.6, true);
                    setPathState(PathState.SHOOT5);
                }
                break;

            case SHOOT5:
                if(!follower.isBusy()){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered){
                        shooter.fireShots(3);
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
        follower = Constants.createFollower(hardwareMap);
        //TODO ADD ANY OTHER INIT STUFF (FLYWHEEL, LIMELIGHT, ETC.)
        // shooter.init(hardwareMap);
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
        //shooter.update();
        StatePathUpdate();




        telemetry.addData("Path State:", pathState.toString());
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path time:", pathTimer.getElapsedTimeSeconds());




    }
}
