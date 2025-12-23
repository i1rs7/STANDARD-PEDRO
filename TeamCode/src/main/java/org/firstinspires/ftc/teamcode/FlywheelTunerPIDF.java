package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTunerPIDF extends OpMode {
    public DcMotorEx outtakeLeft;
    public DcMotorEx outtakeRight;

    public double highVelocity = 950;
    public double lowVelocity = 800;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0,1.0,0.1,0.001,0.0001};

    int stepIndex = 1;


    @Override
    public void init(){
        outtakeLeft = hardwareMap.get(DcMotorEx.class,"oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class,"oR");

        outtakeLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop (){
        //get all gamepad commands
        //set target velocity
        //update telemetry

        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else{curTargetVelocity = highVelocity;}

        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex+1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P -= stepSizes[stepIndex];
        }

        // set new pidf coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        // set velocity
        outtakeLeft.setVelocity(curTargetVelocity);
        outtakeRight.setVelocity(curTargetVelocity);

        double curVelocity = outtakeLeft.getVelocity();
        double error = curTargetVelocity-curVelocity;

        telemetry.addData("Target Velocity",curTargetVelocity);
        telemetry.addData("Current Velocity",curVelocity);
        telemetry.addData("Error",error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Tuning P (D-Pad U/D", P);
        telemetry.addData("Tuning F (D-Pad L/R", F);
        telemetry.addData("Step Size (B Button)", stepSizes[stepIndex]);
    }
}
