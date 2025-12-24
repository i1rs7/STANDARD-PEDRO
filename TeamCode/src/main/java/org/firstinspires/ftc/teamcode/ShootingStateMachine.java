package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingStateMachine {
    private ElapsedTime stateTimer = new ElapsedTime();

    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;
    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }

    private FlywheelState flywheelState;

    // ------ FLYWHEEL CONSTANTS -------

    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 800;
    private double TARGET_FLYWHEEL_RPM = 1100;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2;

    private void init(){

        // tune pidf
        // set to run using encoder

        flywheelState = FlywheelState.IDLE;


    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    //lower the balls a little
                    //set velocity

                }
        }
    }
}
