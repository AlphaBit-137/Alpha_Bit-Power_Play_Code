package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Useful_Methods;

import java.util.Arrays;
import java.util.List;

public class  TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.885; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X =Useful_Methods.CmToInch(-12); // X is the up and down direction
    public static double PARALLEL_Y = Useful_Methods.CmToInch(7.45); // Y is the strafe direction

    public static double PERPENDICULAR_X = Useful_Methods.CmToInch(11.9);
    public static double PERPENDICULAR_Y = 0;

    public static double X_MULTIPLIER = 1.329405314745742; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.329405314745742; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
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

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front_Left"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front_Right"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
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
