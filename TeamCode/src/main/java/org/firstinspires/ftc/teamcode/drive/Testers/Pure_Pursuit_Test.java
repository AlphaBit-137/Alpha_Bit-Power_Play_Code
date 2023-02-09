package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;


/**
 * version 0.1
 *
 * Would make better versions but im tired
 */

@TeleOp
public class Pure_Pursuit_Test extends LinearOpMode {

    Gyroscope gyro = new Gyroscope();
    ChasisInit cs = new ChasisInit();

    double x;
    double y;
    double theta;

    double targetX;
    double targetY;
    double targetTheta;

    Pid_Controller PidX;
    Pid_Controller PidY;
    Pid_Controller PidTheta;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        gyro.Init(hardwareMap);
        cs.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        PidX = new Pid_Controller(0.005,0,0);
        PidY = new Pid_Controller(0.005,0,0);
        PidTheta = new Pid_Controller(0.005,0,0);

        waitForStart();

        while(opModeIsActive())
        {
            gyro.updateOrientation();

            double currentAngle = gyro.getHeading();
            double currentRadAngle = Math.toRadians(currentAngle);
            double currentX = 0;
            double currentY = 0;



            x = PidX.returnPower(currentX , targetX);
            y = PidY.returnPower(currentY , targetY);
            theta = PidTheta.returnPower(Math.toDegrees(angleWrap(currentRadAngle)),targetTheta);

            double x_rotated = x * Math.cos(currentRadAngle) - y * Math.sin(currentRadAngle);
            double y_rotated = x * Math.sin(currentRadAngle) + y * Math.cos(currentRadAngle);

            cs.FrontLeft.setPower(x_rotated + y_rotated + theta);
            cs.BackLeft.setPower(x_rotated - y_rotated + theta);
            cs.FrontRight.setPower(x_rotated - y_rotated - theta);
            cs.BackRight.setPower(x_rotated + y_rotated - theta);

        }

    }


    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
}
