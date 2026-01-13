package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class FlywheelLogic {

    public ElapsedTime stateTimer = new ElapsedTime();

    private Servo door;
    private DcMotor intakeMotor;
    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;
    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }

    public FlywheelState flywheelState;

    // gate constants
    private double GATE_CLOSE_ANGLE = 0.1; //todo find this value
    private double GATE_OPEN_ANGLE = 0.5; // todo find this value
    private double GATE_OPEN_TIME = 0.4; // todo find this value
    private double GATE_CLOSE_TIME = 0.4; // todo find this value

    // ------ SHOOTER CONSTANTS --------
    // this servo does not exist yet on robot v1, may exist in v2 so pre-coded
    //private double shooterResetAngle = 0; //placeholder values (0, 90, 0.5)
    //private double shooterShootAngle = 90;
    private double timeToLower = 1.0;
    private double timeToShoot = 0.5; //amount of time the shooting takes


    // ------ FLYWHEEL CONSTANTS -------

    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double CLOSE_FLYWHEEL_RPM = 800;
    private double FAR_FLYWHEEL_RPM = 950;
    private double target_range = 20;
    private double TARGET_FLYWHEEL_RPM = CLOSE_FLYWHEEL_RPM;
    private double FLYWHEEL_MAX_SPINUP_TIME = 3; //safety check in case flywheel takes forever


    public void init(HardwareMap hardwareMap){

        door = hardwareMap.get(Servo.class, "d");

        intakeMotor = hardwareMap.get(DcMotor.class,"i");
        outtakeLeft = hardwareMap.get(DcMotorEx.class,"oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class,"oR");

        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15,0,0,13);
        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // intake up
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;

        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
        door.setPosition(GATE_CLOSE_ANGLE);
    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    door.setPosition(GATE_CLOSE_ANGLE);
                    outtakeLeft.setVelocity(TARGET_FLYWHEEL_RPM);
                    outtakeRight.setVelocity(TARGET_FLYWHEEL_RPM);
                    intakeMotor.setPower(0);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (FlywheelsAtSpeed() || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME){
                    door.setPosition(GATE_OPEN_ANGLE);
                    intakeMotor.setPower(0.95);

                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:

                if (stateTimer.seconds() > GATE_OPEN_TIME){
                    shotsRemaining --;

                    door.setPosition(GATE_CLOSE_ANGLE);

                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;

            case RESET:
                if (stateTimer.seconds() > GATE_CLOSE_TIME){
                    if (shotsRemaining > 0){
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }
                    else{
                        outtakeLeft.setPower(0);
                        outtakeRight.setPower(0);
                        intakeMotor.setPower(0);

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

    public boolean flywheelsAreBusy(){
        return flywheelState != FlywheelState.IDLE;
    }
}
