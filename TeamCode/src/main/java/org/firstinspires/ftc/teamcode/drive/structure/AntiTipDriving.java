package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class AntiTipDriving {

    ChasisInit chasis = new ChasisInit();
    Gamepad gamepad;
    Gyroscope gyroscope = new Gyroscope();

    // Speed Multiplier
    double speed = 1.0;
    // If the Joystick has a lower value than this one the robot will not move
    final double controllerDeadzone = 0.15;
    // If the robot should move in reverse or not
    double reverse = 1.0;

    double Kp = 0.08, Ki = 0, Kd = 0;
    Pid_Controller forwardAnglePID;
    Pid_Controller lateralAnglePID;
    Pid_Controller HeadingPid;

    double forwardAngleValuePID = 0;
    double lateralAngleValuePID = 0;
    double headingValuePID = 0;

    public void init(Gamepad gamepad, HardwareMap hwmap)
    {
        this.gamepad = gamepad;
        chasis.init(hwmap);
        gyroscope.Init(hwmap);

        forwardAnglePID = new Pid_Controller(0.09,0.0,0);
        lateralAnglePID = new Pid_Controller(0.09,0.0,0);
        HeadingPid = new Pid_Controller(0.05,0.0,0);

        gyroscope.updateOrientation();
        gyroscope.initFirstAngles();
    }

    public void run() {

        gyroscope.updateOrientation();

        forwardAngleValuePID = forwardAnglePID.returnPower(gyroscope.firstForward,
                gyroscope.getForwardAngle());
        lateralAngleValuePID = lateralAnglePID.returnPower(gyroscope.firstLateral,
                gyroscope.getLateralAngle());
        headingValuePID = HeadingPid.returnPower(gyroscope.firstHeading,gyroscope.getHeading());

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        if(x != 0)
        {
            gyroscope.updateFirstLateral();
        }

        if(y != 0)
        {
            gyroscope.updateFirstForward();
        }

        if(r != 0)
        {
            gyroscope.updateFirstForward();
            gyroscope.updateFirstAngles();
            gyroscope.updateFirstLateral();
        }


        double x2 = gamepad.left_stick_x + lateralAngleValuePID; // Strafe, Horizontal Axis
        double y2 = -gamepad.left_stick_y + forwardAngleValuePID; // Forward, Vertical Axis (joystick has +/- flipped)
        double r2 = gamepad.right_stick_x + headingValuePID; // Rotation, Horizontal Axis



        x2 = addons(x2) * reverse;
        y2 = addons(y2) * reverse;
        r2 = addons(r2);

        double normalizer = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r2), 1.0);

        double FrontLeft = (y2 + x2 + r2) / normalizer;
        double FrontRight = (y2 - x2 - r2) / normalizer;
        double BackLeft = (y2 - x2 + r2) / normalizer;
        double BackRight = (y2 + x2 - r2) / normalizer;


        chasis.FrontLeft.setPower(FrontLeft);
        chasis.FrontRight.setPower(FrontRight);
        chasis.BackLeft.setPower(BackLeft);
        chasis.BackRight.setPower(BackRight);


    }

    public double returnHeading()
    {
        return gyroscope.getHeading();
    }

    public double returnLateral()
    {
        return gyroscope.getLateralAngle();
    }

    public double returnForward()
    {
        return gyroscope.getForwardAngle();
    }


    private double addons(double coord) {
        if (Math.abs(coord) < controllerDeadzone) {
            coord = 0;
        }
        coord = coord * speed;

        return coord;
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) {
            reverse = -1.0;
        } else {
            reverse = 1.0;
        }
    }

}
