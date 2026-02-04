package org.firstinspires.ftc.teamcode.limelight;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
public class AprilTagLimelightDistance extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private double distance;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(  1); //obelisk = 0, limelight = 1
        imu = hardwareMap.get(IMU.class,"pinpoint");

    }
    @Override
    public void start(){
        limelight.start();

    }
    @Override
    public void loop(){

        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose();
            double camX  = -botpose.getPosition().x;
            double camY  = botpose.getPosition().y;
            distance = getDistance(camX, camY);
            telemetry.addData("distance", distance);
            telemetry.addData("Cam X", camX);
            telemetry.addData("Cam Y", camY);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
        } else {
            telemetry.addLine("No result");
        }


    }

    public double getDistance(double camX, double camY){
        double mtBlueX = -1.482;
        double mtBlueY = -1.413;
        double mtRedX = 1.482;
        double mtRedY = 1.413;
        distance = Math.sqrt(Math.pow((mtRedX-camX), 2) + Math.pow((mtRedY-camY),2));
        return distance;
    }

}
