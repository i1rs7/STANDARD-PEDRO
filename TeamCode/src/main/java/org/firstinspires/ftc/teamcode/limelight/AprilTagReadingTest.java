package org.firstinspires.ftc.teamcode.limelight;
import android.media.MediaDrm;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Disabled
public class AprilTagReadingTest extends OpMode {

    private Limelight3A limelight;
    private double tagId;

    public double getHeading () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private IMU imu;
    GoBildaPinpointDriver pinpoint;

    private boolean isPinpoint;

    //private double distance;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        imu = hardwareMap.get(IMU.class, "imu");

        if (isPinpoint) {

            double offsetX = 0.5;
            double offsetY = 2.25;

            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            pinpoint.setOffsets(offsetX, offsetY, DistanceUnit.MM);

            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            pinpoint.resetPosAndIMU();

            pinpoint.setPosition(
                    new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0)
            );
        }
    }
    @Override
    public void start(){
        limelight.start();

    }
    @Override
    public void loop(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if(llResult !=null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
        }
        telemetry.update();
    }
}