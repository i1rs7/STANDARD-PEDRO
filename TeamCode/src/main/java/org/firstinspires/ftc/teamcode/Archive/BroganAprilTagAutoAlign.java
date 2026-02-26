package org.firstinspires.ftc.teamcode.Archive;
//package org.firstinspires.ftc.teamcode.tutorials;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Disabled
public class BroganAprilTagAutoAlign extends OpMode {
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final MecanumDrive drive = new MecanumDrive();

    // ------------ PD controller ------------
    double kP = 0.0002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //or add offset here
    double angleTolerance = 0.2;

    double kD = 0.00001;
    double curTime = 0;
    double lastTime = 0;

    // ----- driving setup----------
    double forward, strafe, rotate;

    // ------- controller based pd tuning -------

    double[] stepSizes = {0.1,0.001,0.0001};

    int stepIndex = 1;


    @Override
    public void init (){
        aprilTagWebcam.init(hardwareMap,telemetry); //todo create helper class
        drive.init(hardwareMap,false); //todo create helper class
        telemetry.addLine("initialized");

    }

    public void start(){
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop (){
        // ------- get mecanum drive inputs -------
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // ------- get april tag info -------
        aprilTagWebcam.update(); //todo apriltagwebcam
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);

        // ------- auto align rotation logic -------
        if (gamepad1.left_trigger > 0.3){
            if (id20 != null){
                error = goalX - id20.ftcPose.bearing; //tx

                if (Math.abs(error)< angleTolerance){
                    rotate = 0;
                } else{
                    double pTerm = error *kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error-lastError)/dT) *kD;

                    rotate = Range.clip(pTerm+dTerm,-0.4,0.4);

                    lastError = error;
                    lastTime = curTime;

                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else{
            lastError = 0;
            lastTime = getRuntime();

        }

        // drive our motors
        drive.drive(forward, strafe, rotate);
        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex+1) % stepSizes.length; //goes to next index, then wraps the index back to 0
        }

        //D-pad right increases the P variable
        if(gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        //D-pad left decreases the P variable
        if(gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }

        //D-pad up increases the D variable
        if(gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }

        //D-pad down decreases the D variable
        if(gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        // -------- telemetry ------------
        if (id20 != null){
            if (gamepad1.left_trigger >0.3){
                telemetry.addLine("AUTO ALIGN");
            }
            aprilTagWebcam.displayDetectionTelemetry(id20);
            telemetry.addData("error", error);
        } else{
            telemetry.addLine("MANUAL rotate mode");
        }
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P (D-Pad U/D", kP);
        telemetry.addData("Tuning D (D-Pad L/R", kD);
        telemetry.addData("Step Size (B Button)", stepSizes[stepIndex]);

    }
}
