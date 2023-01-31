package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyroscope {

    public BNO055IMU imu;

    public double firstHeading = 0;
    public double firstLateral = 0;
    public double firstForward = 0;


    boolean firstAngles = false;

    public void Init(HardwareMap hwmap) {

        imu = hwmap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }


    Orientation angularOrientation;
    
    AngularVelocity angularVelocity;


    ElapsedTime oriUpdateTimer = new ElapsedTime();

    public void updateOrientation() {
        if (oriUpdateTimer.milliseconds() > 1) {
            angularOrientation = imu.getAngularOrientation();
            oriUpdateTimer.reset();
        }
    }



    ElapsedTime gyroVelTimer = new ElapsedTime();

    public void updateVelocity() {
        if (gyroVelTimer.milliseconds() > 1) {
            angularVelocity = imu.getAngularVelocity();
            gyroVelTimer.reset();
        }
    }


    public double getHeading() {
        return angularOrientation.firstAngle;
    }

    public double getForwardAngle() {
        return angularOrientation.secondAngle;
    }

    public double getLateralAngle() {
        return angularOrientation.thirdAngle;
    }

    public Orientation AngularOrientation()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



}

