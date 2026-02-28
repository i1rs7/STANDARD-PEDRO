package org.firstinspires.ftc.teamcode.OldSuperQualAuto;


//import static java.lang.Thread.sleep;


//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
public class RedClose12LEVER {
/*extends OpMode {


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
        DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE,

        STARTINTAKE2,

        //Intake second set of three + move balls down + start flywheels
        DRIVE_CONTROLLEVERPOSE_LEVERPOSE,

        STOPINTAKE2,

        //Go to first spot before shooting position to avoid the lever
        DRIVE_LEVERPOSE_SHOOTPOSE3,

        //Go to shooting position and shoot next 3

        SHOOT3,

        //Line up to intake next 3 balls
        DRIVE_SHOOTPOSE3_LINEINTAKE2POSE,
        STARTINTAKE3,

        DRIVE_LINEINTAKE2POSE_INTAKE2POSE,

        STOPINTAKE3,

        DRIVE_INTAKE2POSE_SHOOTPOSE4,

        SHOOT4,

        DONE
    }


    PathState pathState;

    RedAutoClose12.PathState pathState;

    //all points
    private final Pose startPose = new Pose(145-33.4555712270804, 136.1579689703808, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(145-55.85680170543313, 92.07533215512152, Math.toRadians(47));
    private final Pose shootPose2 = new Pose(145-53.856801705433135, 87.34008052590521, Math.toRadians(48));
    private final Pose shootPose3 = new Pose(145-50.856801705433135, 87.34008052590521, Math.toRadians(46));
    private final Pose shootPose4 = new Pose(145-48, 85, Math.toRadians(44));

    //make sure this on is inside the zone to get the leave points
    // private final Pose shootPose5 = new Pose(145-35, 110, Math.toRadians(36));



    private final Pose lineIntake1Pose = new Pose(145-56.97981157469717, 87.07533215512152, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(145-21.197060671580733, 87.07533215512152, Math.toRadians(180));
    private final Pose controlLeverPose = new Pose (145-35.917366981341605, 68.60157710801516, Math.toRadians(220));
    private final Pose leverPose = new Pose(145-13.54823695345557, 89.60157710801516, Math.toRadians(220));
    private final Pose lineIntake2Pose = new Pose(145-56.85680170543313, 66.524682651622, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(145-12.08416494712284, 66.524682651622, Math.toRadians(180));

//    private final Pose controlLever2 = new Pose (145-35.917366981341605, 68.60157710801516, Math.toRadians(220));
//    private final Pose leverPose2 = new Pose(145-13.54823695345557, 68.60157710801516, Math.toRadians(220));



    //All the movement paths (no intake/outtake)
    private PathChain driveStartPosShootPos1, driveShootPos1LineIntake1Pos, driveLineIntake1PosIntake1Pos, driveIntake1PosShootPos2,
            driveShootPos2ControlLeverPos, driveControlLeverPosLeverPos, driveLeverPosShootPos3, driveShootPos3LineIntake2Pos, driveLineIntake2Intake2Pos, driveIntake2PosShootPos4;
















    public void buildPaths(){
        //use coordinates of the points (above) for the starting position and the ending position to construct a path
        driveStartPosShootPos1 = follower.pathBuilder()
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
        driveShootPos2ControlLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, controlLeverPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), controlLeverPose.getHeading())
                .build();
        driveControlLeverPosLeverPos = follower.pathBuilder()
                .addPath(new BezierLine(controlLeverPose, leverPose))
                .setLinearHeadingInterpolation(controlLeverPose.getHeading(), leverPose.getHeading())
                .build();
        driveLeverPosShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(leverPose, shootPose3))
                .setLinearHeadingInterpolation(leverPose.getHeading(), shootPose3.getHeading())
                .build();
        driveShootPos3LineIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, lineIntake2Pose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), lineIntake2Pose.getHeading())
                .build();
        driveLineIntake2Intake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(lineIntake1Pose, intake2Pose))
                .setLinearHeadingInterpolation(lineIntake1Pose.getHeading(), intake2Pose.getHeading())
                .build();
        driveIntake2PosShootPos4 = follower.pathBuilder()
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
                outtakeLeft.setVelocity(770);
                outtakeRight.setVelocity(770);
                follower.followPath(driveStartPosShootPos1, 0.9, true); //Follow the path
                setPathState(RedCloseAuto.PathState.SHOOT1); //RESET TIMER & SET TO NEXT PATH STATE
                telemetry.addLine("Moved back");
                break;

             case SHOOT1:
                telemetry.addLine("In SHOOT1");

                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered && pathTimer.getElapsedTimeSeconds() > 2){
                        shooter.fireShots(5);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(RedCloseAuto.PathState.DRIVE_SHOOTPOSE1_LINEINTAKE1POSE);
                    }
                } break;


            case DRIVE_SHOOTPOSE1_LINEINTAKE1POSE:
                telemetry.addLine("In Drive_SP1_LI1P");
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake first set of balls");
                    follower.followPath(driveShootPos1LineIntake1Pos, 0.8, true);
                    setPathState(RedCloseAuto.PathState.STARTINTAKE1);
                }
                break;

            case STARTINTAKE1:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    //shootMotor.setPower(0.95);
                    telemetry.addLine("Started intake to intake first 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_LINEINTAKE1POSE_INTAKE1POSE);
                }
                break;


            case DRIVE_LINEINTAKE1POSE_INTAKE1POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook 3 balls");
                    follower.followPath(driveLineIntake1PosIntake1Pos, 0.5,true);
                    setPathState(RedCloseAuto.PathState.STOPINTAKE1);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy()) {
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_INTAKE1POSE_SHOOTPOSE2);
                }
                break;

            case STOPINTAKE1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3) {
                    telemetry.addLine("Stopped intake after intaked first 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_INTAKE1POSE_SHOOTPOSE2);
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                }
                break;


            case DRIVE_INTAKE1POSE_SHOOTPOSE2:
                if(!follower.isBusy()){
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake1PosShootPos2, 0.7,true);
                    setPathState(RedCloseAuto.PathState.SHOOT2);
                }
                break;


            case SHOOT2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5 ){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered && pathTimer.getElapsedTimeSeconds() > 2 ){
                        shooter.fireShots(4);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(RedCloseAuto.PathState.DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1);
                    }
                }
                break;


            case DRIVE_SHOOTPOSE2_CONTROLLEVERPOSE1:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPos2ControlLeverPose1, 0.8,true);
                    setPathState(RedCloseAuto.PathState.STARTINTAKE2);
                }
                break;


            case STARTINTAKE2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    //shootMotor.setPower(0.95);
                    //TODO add intake logic to start intake
                    telemetry.addLine("Started intake to intake second 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_CONTROLLEVERPOSE1_LEVERPOSE1);
                }
                break;


            case DRIVE_CONTROLLEVERPOSE_LEVERPOSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook second set of balls");
                    follower.followPath(driveControlLeverPosLeverPos, 0.6, true);
                    setPathState(RedCloseAuto.PathState.STOPINTAKE2);
                }
                break;


            case STOPINTAKE2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add intake logic to stop intake
                    //TODO add intake logic to move balls down slightly
                    //TODO start flywheels
                    telemetry.addLine("Stopped intake after intaked second 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_LEVERPOSE1_SHOOTPOSE3);
                }
                break;



            case DRIVE_LEVERPOSE_SHOOTPOSE3:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    telemetry.addLine("Moved to preliminary lever pos ");
                    follower.followPath(driveLeverPosShootPos3, 0.6, true);
                    setPathState(RedCloseAuto.PathState.SHOOT3);
                }
                break;


            case SHOOT3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered && pathTimer.getElapsedTimeSeconds() > 2){
                        shooter.fireShots(4);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot first 3");
                        setPathState(RedCloseAuto.PathState.DRIVE_SHOOTPOSE3_CONTROLLEVERPOSE2);
                    }
                }
                break;

            case DRIVE_SHOOTPOSE3_LINEINTAKE2POSE:
                if(!follower.isBusy()){
                    door.setPosition(GATE_UP_ANGLE);
                    telemetry.addLine("Lined up to intake second set of balls");
                    follower.followPath(driveShootPos3LineIntake2Pos, 0.8, true);
                    setPathState(RedCloseAuto.PathState.STARTINTAKE3);
                }
                break;

            case STARTINTAKE3:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0.95);
                    shootMotor.setPower(0.95);
                    telemetry.addLine("Started intake to intake third 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_LINEINTAKE2POSE_INTAKE2POSE);
                }
                break;

            case DRIVE_LINEINTAKE2POSE_INTAKE2POSE:
                if(!follower.isBusy()){
                    telemetry.addLine("Intook third set of balls");
                    follower.followPath(driveLineIntake2Intake2Pos, 0.6, true);
                    setPathState(RedCloseAuto.PathState.STOPINTAKE3);
                }
                break;

            case STOPINTAKE3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    telemetry.addLine("Stopped intake after intaked third 3");
                    setPathState(RedCloseAuto.PathState.DRIVE_INTAKE2POSE_SHOOTPOSE4);
                }
                break;

            case DRIVE_INTAKE2POSE_SHOOTPOSE4:
                if(!follower.isBusy()){
                    intakeMotor.setPower(0);
                    shootMotor.setPower(0);
                    telemetry.addLine("Moved to shooting position and shot next 3 balls");
                    follower.followPath(driveIntake2PosShootPos4, 0.9, true);
                    setPathState(RedCloseAuto.PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    door.setPosition(GATE_DOWN_ANGLE);
                    if (!shotsTriggered && pathTimer.getElapsedTimeSeconds() > 2){
                        shooter.fireShots(4);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.flywheelsAreBusy()){
                        //shots are done, free to transition
                        telemetry.addLine("Shot next 3");
                        setPathState(RedCloseAuto.PathState.DONE);
                    }
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




















    public void setPathState (RedCloseAuto.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;

    }




    @Override
    public void init() {
        pathState = RedCloseAuto.PathState.DRIVE_STARTPOSE_SHOOTPOSE1; //Whats the difference between DRIVE_STARTPOSE_SHOOTPOSE and driveStartPosShootPos
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
        outtakeLeft.setVelocity(770);
        outtakeRight.setVelocity(770);
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
   */
}
