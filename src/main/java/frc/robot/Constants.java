// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean DEBUG = true;

    // Motor channel constants
    // Chassis (FL: Front Left FR: Front Right RL: Rear Left RR: Rear Right)
    public static final int MOTOR_CHASSIS_FL = 1;
    public static final int MOTOR_CHASSIS_FR = 2;
    public static final int MOTOR_CHASSIS_RL = 3;
    public static final int MOTOR_CHASSIS_RR = 4;
    // Hang
    public static final int MOTOR_HANG_LEFT = 5;
    public static final int MOTOR_HANG_RIGHT = 6;
    // Shoot
    public static final int MOTOR_SHOOT_MAIN_UP = 11;
    public static final int MOTOR_SHOOT_MAIN_DOWN = 12;
    public static final int MOTOR_SHOOT_ROTATE = 10;
    public static final int MOTOR_SHOOT_ANGLE = 11;
    public static final int MOTOR_SHOOT_TRIGGER = 7;
    // Collect
    public static final int MOTOR_TRANSFER = 8;
    public static final int MOTOR_INTAKE = 9;

    // Motor Invert constants
    // Chassis
    public static final boolean MOTOR_CHASSIS_FL_INVERTED = true;
    public static final boolean MOTOR_CHASSIS_FR_INVERTED = false;
    public static final boolean MOTOR_CHASSIS_RL_INVERTED = true;
    public static final boolean MOTOR_CHASSIS_RR_INVERTED = false;
    // Hang
    public static final boolean MOTOR_HANG_LEFT_INVERTED = false;
    public static final boolean MOTOR_HANG_RIGHT_INVERTED = true;
    // Shoot
    public static final boolean MOTOR_SHOOT_UP_INVERTED = false;
    public static final boolean MOTOR_SHOOT_DOWN_INVERTED = true;
    public static final boolean MOTOR_SHOOT_ROTATE_INVERTED = false;
    public static final boolean MOTOR_SHOOT_ANGLE_INVERTED = true;
    public static final boolean MOTOR_SHOOT_TRIGGER_INVERTED = false;
    // Intake
    public static final boolean MOTOR_INTAKE_INVERTED = false;
    public static final boolean MOTOR_TRANSFER_INVERTED = false;


    // Chassis motors' current limit
    public static final int CURRENT_LIMIT_CHASSIS = 37;

    // Solenoid channel constants
    public static final int SOLENOID_HANG_FORWARD = 1;
    public static final int SOLENOID_HANG_REVERSE = 0;
    public static final int SOLENOID_INTAKE_FORWARD = 4;
    public static final int SOLENOID_INTAKE_REVERSE = 5;

    // Chassis constants
    public static final boolean ENABLE_FOD = true;
    public static final double CHASSIS_DEADBAND = 0.06;
    public static final double CHASSIS_GEARING = 14.285;
    public static final double DISTANCE_METER_PER_ROTATION = 6.3 * 2.54 / 100.0 * Math.PI; // Wheel Diameter(Inches) * 2.54(inch to cm) / 100.0(cm to meter) * math.PI(Diameter to Wheel Perimeter)
    public static final double MAX_CHASSIS_VELOCITY_METER_PER_SECOND = 6;
    public static final double MAX_CHASSIS_ACCELERATION_METER_PETER_SECOND_SQUARE = 1;
    public static final double WIDTH = 0.566;
    public static final double LENGTH = 0.507;
    public static final CANSparkMax.IdleMode IDLE_MODE = CANSparkMax.IdleMode.kCoast;

    // FeedForward constants for motors
    public static final double FF_SHOOT_MAIN = 0.051;
    public static final double FF_SHOOT_ROTATE = 0.00479; // velocity p=0.001 maxV=200 maxA=175
    public static final double FF_SHOOT_ANGLE = 0.000093;
    public static final double FF_SHOOT_TRIGGER = 0.000089661;
    public static final double FF_SHOOT_TRANSFER = 0.00017696;
    public static final double FF_HANG = 0.002;
    public static final double FF_CHASSIS_KS = 0.15085;
    public static final double FF_CHASSIS_KV = 3.7072; // unit is voltage/meter per second
    public static final double FF_CHASSIS_KA = 0.34931; // unit is voltage/meter per second^2

    // PID constants for motors
    public static final double[] PID_CHASSIS = {1.8833, 0, 0}; // From sys-id
    public static final double[] PID_HANG = {0.00025, 0, 0};
    public static final double[] PID_SHOOT_MAIN = {0.07, 0, 0};
    public static final double[] PID_SHOOT_ROTATE = {0.001, 0, 0};
    public static final double[] PID_SHOOT_ANGLE = {0.00007, 0, 0};
    public static final double[] PID_SHOOT_TRIGGER = {0, 0, 0};
    public static final double[] PID_SHOOT_TRANSFER = {0, 0, 0};

    // Smart Motion
    public static final double SMART_MOTION_MAX_VELOCITY_SHOOT_ROTATE = 200;
    public static final double SMART_MOTION_MAX_ACCEL_SHOOT_ROTATE = 175;
    public static final double SMART_MOTION_MAX_VELOCITY_SHOOT_ANGLE = 12000;
    public static final double SMART_MOTION_MAX_ACCEL_SHOOT_ANGLE = 20000;
    public static final double SMART_MOTION_MAX_VELOCITY_HANG = 3000;
    public static final double SMART_MOTION_MAX_ACCEL_HANG = 1000;

    // Auto alignment
    public static final double AUTO_ALIGNMENT_MOUNT_ANGLE = 20;
    public static final double AUTO_ALIGNMENT_LENS_HEIGHT_METER = 0.95;
    public static final double AUTO_ALIGNMENT_GOAL_HEIGHT_METER = 2.5;

    public static final float SHOOT_MAX_ROTATE_ANGLE = 310;

    public static final double SHOOT_ROTATE_GEARING = 53.6;
    public static final float SOFT_LIMIT_FWD_SHOOT_ANGLE = 15;

    // Addressable led port
    public static final int LED_PWM_PORT = 9;
    public static final int LED_LENGTH = 148;

    public static final double ZERO_ANGLE_MAX_SECONDS = 2;
}
