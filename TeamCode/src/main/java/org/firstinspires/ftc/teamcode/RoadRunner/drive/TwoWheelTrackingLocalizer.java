package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Useful_Methods;

import java.util.Arrays;
import java.util.List;


@Config
public class  TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.68897637795; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

  /*  public static double PARALLEL_X = Useful_Methods.CmToInch(-1.7); // X is the up and down direction
    public static double PARALLEL_Y = Useful_Methods.CmToInch(-9.6); // Y is the strafe direction

    public static double PERPENDICULAR_X = Useful_Methods.CmToInch(-6.6);
    public static double PERPENDICULAR_Y = Useful_Methods.CmToInch(-0.1);*/

    public static double PARALLEL_X = Useful_Methods.CmToInch(1.7); // X is the up and down direction
    public static double PARALLEL_Y = Useful_Methods.CmToInch(-9.6); // Y is the strafe direction

    public static double PERPENDICULAR_X = Useful_Methods.CmToInch(6.6);
    public static double PERPENDICULAR_Y = Useful_Methods.CmToInch(0.1);

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    private Encoder parallelEncoder, perpendicularEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive,List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front_Right"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front_Left"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
       // perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
      //  parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getParallelPosition()
    {
        return parallelEncoder.getCurrentPosition();
    }

    public double getPerpendicularPoition()
    {
        return perpendicularEncoder.getCurrentPosition();
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int paralelPos = parallelEncoder.getCurrentPosition();
        int perpendicularPos = perpendicularEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(paralelPos);
        lastEncPositions.add(perpendicularPos);

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        int paraleltVel = (int) parallelEncoder.getCorrectedVelocity();
        int perpendicularVel = (int) perpendicularEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(paraleltVel);
        lastEncVels.add(perpendicularVel);

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }
}
