package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchStateMachine {

    private LinearOpMode opMode;
    private Telemetry telemetry;

    private ElapsedTime stateTimer = new ElapsedTime();

    //private Servo shootServo;
    private DcMotor intakeMotor;
    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;
    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }

    private FlywheelState flywheelState;

    // ------ SHOOTER CONSTANTS --------
    // this servo does not exist yet on robot v1, may exist in v2 so pre-coded
    //private double shooterResetAngle = 0; //placeholder values (0, 90, 0.5)
    //private double shooterShootAngle = 90;
    private double shootInches = 3;
    private double lowerInches = -1;
    private double timeToShoot = 0.5; //amount of time the shooting takes

    //intake counts per revolution conversion to distance
    private double     ARTIFACT_DIAMETER = 5.0;
    private double     INTAKE_COUNTS_PER_MOTOR_REV    = 384.5 ; //
    private double     INTAKE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    private double     INTAKE_WHEEL_DIAMETER_INCHES   = 1.88976 ;     // For figuring circumference
    private double     INTAKE_COUNTS_PER_INCH         = (INTAKE_COUNTS_PER_MOTOR_REV * INTAKE_GEAR_REDUCTION) /
            (INTAKE_WHEEL_DIAMETER_INCHES * 3.1415);
    private double     INTAKE_TO_ARTIFACT_DISTANCE_CONVERSION = ARTIFACT_DIAMETER/INTAKE_WHEEL_DIAMETER_INCHES;
    private double     INTAKE_SPEED = 0.95;

    // ------ FLYWHEEL CONSTANTS -------

    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double SHORT_FLYWHEEL_RPM = 800;
    private double FAR_FLYWHEEL_RPM = 950;
    private double target_range = 40;
    private double TARGET_FLYWHEEL_RPM = SHORT_FLYWHEEL_RPM;
    private double FLYWHEEL_MAX_SPINUP_TIME = 5; //safety check in case flywheel takes forever


    public void init(HardwareMap hardwareMap){

        //shootServo = hardwareMap.get(Servo.class, "shoot");
        intakeMotor = hardwareMap.get(DcMotor.class,"i");
        outtakeLeft = hardwareMap.get(DcMotorEx.class,"oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class,"oR");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15,0,0,13);
        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // intake up
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        flywheelState = FlywheelState.IDLE;

        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
        //shootServo.setPosition(shooterResetAngle);


    }
    public void intakeByEncoder(double speed, double Inches, double timeoutS){
        int placeholder;
        int newIntakeTarget;
        // Determine new target position, and pass to motor controller
        newIntakeTarget = intakeMotor.getCurrentPosition() + (int) (Inches * INTAKE_COUNTS_PER_INCH * INTAKE_TO_ARTIFACT_DISTANCE_CONVERSION);
        intakeMotor.setTargetPosition(newIntakeTarget);

        // Turn On RUN_TO_POSITION
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() && intakeMotor.isBusy()){
            telemetry.addData("Running to", " %.7f", Inches);
            telemetry.addData("New Intake Target", newIntakeTarget);
            telemetry.update();
        }

        //Stop all motion:
        intakeMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    outtakeLeft.setVelocity(TARGET_FLYWHEEL_RPM);
                    outtakeRight.setVelocity(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (FlywheelsAtSpeed() ||
                        stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME){
                    intakeByEncoder(INTAKE_SPEED, shootInches, 5.0);

                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > timeToShoot){
                    shotsRemaining --;
                    //shootServo.setPosition(shooterResetAngle);

                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;

            case RESET:
                if (stateTimer.seconds() > timeToShoot){
                    if (shotsRemaining > 0){
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }
                    else{
                        outtakeLeft.setPower(0);
                        outtakeRight.setPower(0);

                        flywheelState = FlywheelState.IDLE;
                    }
                }

                break;
        }
    }


    public boolean FlywheelsAtSpeed(){
        if ((outtakeRight.getVelocity() >= TARGET_FLYWHEEL_RPM-target_range &&
                outtakeRight.getVelocity() <=  TARGET_FLYWHEEL_RPM+target_range) &&
                (outtakeLeft.getVelocity() >= TARGET_FLYWHEEL_RPM-target_range &&
                        outtakeLeft.getVelocity() <= TARGET_FLYWHEEL_RPM+target_range)){
            return true;
        }
        else{
            return false;
        }
    }

    public void fireShots(int numberOfShots){
        if (flywheelState == FlywheelState.IDLE){
            shotsRemaining = numberOfShots;
        }
    }

    public boolean ShooterisBusy(){
        return flywheelState != FlywheelState.IDLE;
    }
}
