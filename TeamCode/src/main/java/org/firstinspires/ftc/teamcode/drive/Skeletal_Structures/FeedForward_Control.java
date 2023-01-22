package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

public class FeedForward_Control {

    public double ks = 0.0;
    public double kg = 0.0;
    public double kv = 0.0;
    public double ka = 0.0;

    public FeedForward_Control(double ks, double kg, double kv, double ka) {
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }


    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public double maxAchievableVelocity(double maxVoltage, double acceleration) {

        return (maxVoltage - ks - kg - acceleration * ka) / kv;
    }

    public double minAchievableVelocity(double maxVoltage, double acceleration) {

        return (-maxVoltage + ks - kg - acceleration * ka) / kv;
    }

    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - kg - velocity * kv) / ka;
    }

    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }
}
