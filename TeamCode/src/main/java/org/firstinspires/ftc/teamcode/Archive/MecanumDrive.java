package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private IMU imu;
    GoBildaPinpointDriver pinpoint;
    private boolean isPinpoint;

    public void init(HardwareMap hwMap, boolean isGobildaPinpointIMU){
        frontLeftDrive = hwMap.get(DcMotorEx.class, "fL");
        backLeftDrive = hwMap.get(DcMotorEx.class, "bL");
        frontRightDrive = hwMap.get(DcMotorEx.class, "fR");
        backRightDrive = hwMap.get(DcMotorEx.class, "bR");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(isGobildaPinpointIMU) {
                isPinpoint = true;
                double offsetX = 0;
                double offsetY = 0;
                double encoderCPR = 2000;
                double encoderWheelCircumference = 2 * Math.PI * 96;

                pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
                pinpoint.setOffsets(offsetX, offsetY, DistanceUnit.MM);
                pinpoint.setEncoderResolution((encoderCPR / encoderWheelCircumference), DistanceUnit.MM);

                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                pinpoint.resetPosAndIMU();

                //Set the location of the robot - the place where the robot starts
                pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));

            } else {
                isPinpoint = false;
                imu = hwMap.get(IMU.class, "imu");

                RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT); // i think that is the direction of the USBs on the control hub

                imu.initialize(new IMU.Parameters(RevOrientation));
            }
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        if (isPinpoint) {
            Pose2D pose2D = getPosition();
            theta = AngleUnit.normalizeRadians(theta - pose2D.getHeading(AngleUnit.RADIANS));
        } else {
            theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public double getHeading () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updatePinpointPosition(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading));
    }

    public void loop(){
        pinpoint.update();
    }
}
