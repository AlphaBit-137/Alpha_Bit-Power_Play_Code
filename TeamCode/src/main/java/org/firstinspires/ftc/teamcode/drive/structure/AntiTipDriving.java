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

    double lastAngle;
    double lastLateral;
    double lastForward;

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

       getLastAngles();
    }

    public void getLastAngles()
    {
        lastAngle = gyroscope.getHeading();

        lastLateral = gyroscope.getLateralAngle();

        lastForward = gyroscope.getForwardAngle();
    }

    public void run() {

        gyroscope.updateOrientation();
        gyroscope.updateAxis();

        forwardAngleValuePID = forwardAnglePID.returnPower(gyroscope.firstForward,
                lastForward);
        lateralAngleValuePID = lateralAnglePID.returnPower(gyroscope.firstLateral,
                lastLateral);
        headingValuePID = HeadingPid.returnPower(gyroscope.firstHeading,lastAngle);

        double x = gamepad.left_stick_x + lateralAngleValuePID;
        double y = -gamepad.left_stick_y + forwardAngleValuePID;
        double r = gamepad.right_stick_x + headingValuePID;


        double x2 = x; // Strafe, Horizontal Axis
        double y2 = y; // Forward, Vertical Axis (joystick has +/- flipped)
        double r2 = r; // Rotation, Horizontal Axis

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
