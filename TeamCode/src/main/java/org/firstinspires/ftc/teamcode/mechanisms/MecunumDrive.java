package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecunumDrive {
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
    }

    public void loop(){

    }
}
