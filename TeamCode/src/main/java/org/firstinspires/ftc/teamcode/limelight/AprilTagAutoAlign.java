package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "AprilTagAutoAlign", group = "Iterative OpMode")
public class AprilTagAutoAlign extends OpMode {

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // --- Constants ---
    private static final double TURN_P = 0.075;
    private static final double TURN_MAX = 0.5;


    @Override
    public void init() {
        // 1. Initialize Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fL");
        frontRight = hardwareMap.get(DcMotor.class, "fR");
        backLeft   = hardwareMap.get(DcMotor.class, "bL");
        backRight  = hardwareMap.get(DcMotor.class, "bR");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // 2. Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(0.5, 2.25, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // Your AprilTag pipeline
        limelight.start();

        telemetry.addLine("Initialized. Press Play.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset position & IMU ONCE when Play is pressed
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        // MUST update every loop
        pinpoint.update();

        // 3. Gamepad Input
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        // 4. Heading
        Pose2D pose = pinpoint.getPosition();
        double currentHeading = pose.getHeading(AngleUnit.RADIANS);

        // 5. Auto-Align

        if (gamepad1.a) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tagYawDegrees = result.getTx();
                // Tx = horizontal offset in degrees

                double error = -Math.toRadians(tagYawDegrees);

                if (Math.abs(tagYawDegrees) > 3.0) { // TODO figure out how many degrees off is ok
                    rx = Range.clip(error * TURN_P, -TURN_MAX, TURN_MAX);
                } else {
                    rx = 0;
                }

                telemetry.addData("Target X", tagYawDegrees);
            }
        }

        // 6. Field-Centric Math
        double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
        double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
        );

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // 7. Motor Power
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Telemetry
        telemetry.addData("Heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Turn Power", rx);
        telemetry.addData("Auto Align", gamepad1.a);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Optional safety
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}