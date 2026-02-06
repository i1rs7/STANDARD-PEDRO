package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagID(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setNumThreads(1)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "limelight"));
        builder.setCameraResolution(new Size(640,480)); //todo: is this size right?
        builder.enableLiveView(false); //reduce latency (??)

        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
        //waiting for cam to be ready before adjustings settings
    }

    private void setManualExposure(int exposureMS, int gain){
        //wait for camera to be open and streaming
        if (visionPortal == null || visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera", "Waiting...");
            //this loop will halt the init process until the camera is ready.
            while (visionPortal != null && visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING){
                try {Thread.sleep(20); } catch (InterruptedException ignored) {}
            }
            telemetry.addData("Camera","ready!");
        }

        //set camera controls
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            try{
                //Get the ExposureControl and GainControl
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

                //Set the exposure mode to Manual
                if (exposureControl.getMode()!= ExposureControl.Mode.Manual){
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50); //give the camera time to switch modes
                }

                //set the exposure and gain
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
                gainControl.setGain(gain);
                Thread.sleep(20);
            } catch(Exception e){
                //handle exceptions, which might occur if the camera is unplugged
                //or the controls are not supported on this camera.
                telemetry.addData("CameraControlError", e.getMessage());
                telemetry.update();
            }
        }

    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedID){
        if (detectedID == null){return;}
        if (detectedID.metadata != null) {
            telemetry.addLine()
        }
    }

}
