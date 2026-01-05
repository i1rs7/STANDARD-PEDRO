package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingStateMachine {
    private ElapsedTime stateTimer = new ElapsedTime();

    private Servo shootServo;
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
    private double shooterResetAngle = 0; //placeholder values (0, 90, 0.5)
    private double shooterShootAngle = 90;
    private double shooterTimeToShoot = 0.5; //amount of time the servo takes to shoot/reset

    // ------ FLYWHEEL CONSTANTS -------

    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 800;
    private double TARGET_FLYWHEEL_RPM = 1100;
    private double FLYWHEEL_MAX_SPINUP_TIME = 5; //safety check in case flywheel takes forever

    private void init(HardwareMap hardwareMap){

        shootServo = hardwareMap.get(Servo.class, "shoot");
        outtakeLeft = hardwareMap.get(DcMotorEx.class,"oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class,"oR");

        // todo tune pidf
        // todo set to run using encoder

        flywheelState = FlywheelState.IDLE;

        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
        shootServo.setPosition(shooterResetAngle);


    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    //shootServo.setPosition(shooterResetAngle); <-- only for v2 robot
                    //lower the balls a little // <-- only for v1 robot
                    outtakeLeft.setPower(TARGET_FLYWHEEL_RPM);
                    outtakeRight.setPower(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:

                break;

            case LAUNCH:
                if (stateTimer.seconds() > shooterTimeToShoot){
                    shotsRemaining --;
                    shootServo.setPosition(shooterResetAngle);

                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;

            case RESET:
                break;


        }
    }
}
