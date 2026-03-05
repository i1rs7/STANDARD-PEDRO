/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Auto Align Standard Drive", group="Linear OpMode")

public class Auto_Align_Standard_Drive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor shootMotor = null;

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    private Servo door = null;


    static final double target_RPM_close = 720;
    static final double target_RPM_far = 950;
    static final double target_range = 40;
    private double TARGET_FLYWHEEL_RPM;

    //auto align init
    private Limelight3A limelight = null;
    public double getHeading () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private IMU imu;
    GoBildaPinpointDriver pinpoint;

    // --- auto align constants ---
    private static final double TURN_P = 0.075;
    private static final double TURN_MAX = 0.5;
    double kP = 0.0002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //or add offset here
    double angleTolerance = 0.2;

    double kD = 0.00001;
    double curTime = 0;
    double lastTime = 0;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables to the config in the control hub
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fR");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "bR");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "i");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "oR");
        door = hardwareMap.get(Servo.class, "d");
        shootMotor = hardwareMap.get(DcMotor.class,"shoot");

        // setting direction for all DcMotors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // intake up
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD); // shooter up

        //brakes
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //auto align setup
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0.5, 2.25, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // Your AprilTag pipeline
        limelight.start();

        //outtake setup
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(36,0,0,19);
        outtakeLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;



            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // END OF DRIVE CODE

            // Intake Code
            if (gamepad2.left_bumper) {
                //intake down
                intakeMotor.setPower(-0.95);
                shootMotor.setPower(-0.95);
            } else if (gamepad2.right_bumper) {
                //intake up
                intakeMotor.setPower(0.95);
                shootMotor.setPower(0.95);
            }
            // shooting code
            else if (gamepad2.b){
                //intake
                intakeMotor.setPower(0.95);
            } else if (gamepad2.y) {
                //shoot down
                shootMotor.setPower(0.95);
            } else {
                shootMotor.setPower(0);
                intakeMotor.setPower(0);
            }

            //outtake code
            if (gamepad2.right_trigger > 0.5) {
                outtakeLeft.setVelocity(target_RPM_close);
                outtakeRight.setVelocity(target_RPM_close);
                TARGET_FLYWHEEL_RPM=target_RPM_close;
            } else if (gamepad2.left_trigger > 0.5) {
                outtakeLeft.setVelocity(target_RPM_far);
                outtakeRight.setVelocity(target_RPM_far);
                TARGET_FLYWHEEL_RPM=target_RPM_far;
            }
            else {
                outtakeLeft.setPower(0);
                outtakeRight.setPower(0);
            }

            //gate code
            if (gamepad2.x) {
                // door up
                door.setPosition(0.60);
            } else if (gamepad2.a) {
                //door down
                door.setPosition(0.30);
            }

        // auto align code
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double currentHeading = pose.getHeading(AngleUnit.RADIANS);

        if (gamepad1.a) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double Tx = result.getTx();
                double error = -Math.toRadians(Tx);
                telemetry.addData("Target X", Tx);
            }
            
            if (Math.abs(error) < angleTolerance) {
                yaw = 0;
            } else {
                // PID calculation
                double pTerm = error * kP;
                curTime = getRuntime();
                double dT = curTime - lastTime;
                double dTerm = ((error - lastError) / dT) * kD;

                yaw = Range.clip(pTerm + dTerm, -0.4, 0.4);

                lastError = error;
                lastTime = curTime;
            }
        } else {
            // Reset PID if button not pressed
            lastError = 0;
            lastTime = getRuntime();
        }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Velocity Left: ", outtakeLeft.getVelocity());
            telemetry.addData("Velocity Right: ", outtakeRight.getVelocity());
            telemetry.addData("Auto Align", gamepad1.a);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Error", error);
            telemetry.addData("Door Position", door.getPosition());
            telemetry.update();

        }
    }
}
