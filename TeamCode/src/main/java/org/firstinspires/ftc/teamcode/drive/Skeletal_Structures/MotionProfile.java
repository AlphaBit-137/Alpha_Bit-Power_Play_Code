package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

public class MotionProfile {

        public double math_normalizer;
        public double distance_normalizer;

        public double motion_profile(double max_acceleration, double max_velocity, double distance, double current_dt) {

            double acceleration_dt = max_velocity / max_acceleration;

            double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

            if (acceleration_distance > (distance / 2)) {

                math_normalizer = distance / max_acceleration;

                if (math_normalizer < 0) {
                    math_normalizer *= -1;
                    distance_normalizer = -1;
                } else {
                    distance_normalizer = 1;
                }

                acceleration_dt = Math.sqrt(math_normalizer);
            }

            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);


            max_velocity = max_acceleration * acceleration_dt;


            double deacceleration_dt;
            deacceleration_dt = acceleration_dt;

            double cruise_distance;
            cruise_distance = distance - acceleration_distance;
            double cruise_dt;
            cruise_dt = cruise_distance / max_velocity;
            double deacceleration_time;
            deacceleration_time = acceleration_dt + cruise_dt;

            double entire_dt;
            entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
            if (current_dt > entire_dt)
                return distance;

            double cruise_current_dt;
            if (current_dt < acceleration_dt) {
                return (0.5 * max_acceleration * Math.pow(current_dt, 2)) * distance_normalizer;
            } else if (current_dt < deacceleration_time) {
                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
                cruise_current_dt = current_dt - acceleration_dt;
                return (acceleration_distance + max_velocity * cruise_current_dt) * distance_normalizer;
            } else {

                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
                cruise_distance = max_velocity * cruise_dt;
                deacceleration_time = current_dt - deacceleration_time;


                return (acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * Math.pow(deacceleration_time, 2)) * distance_normalizer;
            }
        }

    }




