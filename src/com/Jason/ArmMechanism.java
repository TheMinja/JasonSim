package com.Jason;


import static com.Jason.ArmMechanism.Motors.Bag.*;
import static com.Jason.ArmMechanism.Motors.Pro.MOTOR_KT;

/**
 * For Arm Systems with most of the weight at the end of the arm
 *
 * @author Jason Stanley
 */
public class ArmMechanism {

    private final MotorType motor;
    private double acceleration;
    private double gearRatio;
    private double position;
    private double velocity;
    private double lengthOfArm;
    private int motorCount;
    private double massOnSystem;

    private double torqueConstant = Motors.Pro.MOTOR_KT;
    private double currentVoltage;
    private double velocityConstant = Motors.Pro.MOTOR_KV;
    private double motorResistance = Motors.Pro.RESISTANCE;


    public ArmMechanism(double gearRatio, MotorType motor, int motorCount, double massOnSystem, double lengthOfArm) {
        this.lengthOfArm = lengthOfArm;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.motorCount = motorCount;
        this.massOnSystem = massOnSystem;
        this.torqueConstant =  torqueConstant * motorCount;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void updateVoltage(double voltage) {
        currentVoltage = voltage;
    }

    public double calculateAcceleration() {
        acceleration = ((gearRatio * torqueConstant * currentVoltage)

                - ((1 / velocityConstant) * torqueConstant * velocity * gearRatio * gearRatio))

                / (motorResistance  * massOnSystem * Math.pow(lengthOfArm, 2));

        return acceleration;
    }

    /**
     * Integrates over the Acceleration to find how much our velocity has changed in the past interval.
     *
     * @param intervalInMs
     */
    public double calculateVelocity(int intervalInMs) {
        velocity += .001*(intervalInMs) * calculateAcceleration();
        return velocity;
    }

    /**
     * Integrates over the Velocity to find how much our position has changed in the past interval.
     *
     * @param intervalInMs
     */
    public double calculatePosition(int intervalInMs) {
        position += .001*(intervalInMs) * velocity;
        return position;
    }

    public enum MotorType {
        BAG(MOTOR_KV, Motors.Bag.MOTOR_KT, RESISTANCE),
        CIM(Motors.Cim.MOTOR_KV, Motors.Cim.MOTOR_KT, Motors.Cim.RESISTANCE),
        MINI_CIM(Motors.MiniCim.MOTOR_KV, Motors.MiniCim.MOTOR_KT, Motors.MiniCim.RESISTANCE),
        PRO(Motors.Pro.MOTOR_KV, MOTOR_KT, Motors.Pro.RESISTANCE);

        public final double velocityConstant;
        public final double torqueConstant;
        public final double motorResistance;

        MotorType(double velocityConstant, double torqueConstant, double motorResistance) {
            this.velocityConstant = velocityConstant;
            this.torqueConstant = torqueConstant;
            this.motorResistance = motorResistance;
        }
    }

    public abstract class Motors {

        public abstract class Pro {
            public static final double FREE_SPEED_RPM = 18730.;
            public static final double FREE_CURRENT = .7;
            public static final double MAXIMUM_POWER = 347;
            public static final double STALL_TORQUE = 6.28;
            public static final double STALL_CURRENT = 134;
            public static final double MOTOR_KT = STALL_TORQUE / STALL_CURRENT;
            public static final double RESISTANCE = 12 / STALL_CURRENT;
            public static final double MOTOR_KV = ((FREE_SPEED_RPM / 60) * 360d)
                    / (12 - RESISTANCE * FREE_CURRENT);
        }

        public abstract class Bag {
            public static final double FREE_SPEED_RPM = 13180;
            public static final double FREE_CURRENT = 1.8;
            public static final double MAXIMUM_POWER = 149;
            public static final double STALL_TORQUE = 3.81;
            public static final double STALL_CURRENT = 53;
            public static final double MOTOR_KT = STALL_TORQUE / STALL_CURRENT;
            public static final double RESISTANCE = 12 / STALL_CURRENT;
            public static final double MOTOR_KV = ((FREE_SPEED_RPM / 60) * Math.PI * 2)
                    / (12 - RESISTANCE * FREE_CURRENT);
        }

        public abstract class Cim {
            public static final double FREE_SPEED_RPM = 5330;
            public static final double FREE_CURRENT = /* 2.7 */1.17;
            public static final double MAXIMUM_POWER = 337;
            public static final double STALL_TORQUE = 21.33;
            public static final double STALL_CURRENT = 131;
            public static final double MOTOR_KT = STALL_TORQUE / STALL_CURRENT;
            public static final double RESISTANCE = 12 / STALL_CURRENT;
            public static final double MOTOR_KV = ((FREE_SPEED_RPM / 60) * Math.PI * 2)
                    / (12 - RESISTANCE * FREE_CURRENT);
        }

        public abstract class MiniCim {
            public static final double FREE_SPEED_RPM = 5840;
            public static final double FREE_CURRENT = 3;
            public static final double MAXIMUM_POWER = 215;
            public static final double STALL_TORQUE = 12.48;
            public static final double STALL_CURRENT = 89;
            public static final double MOTOR_KT = STALL_TORQUE / STALL_CURRENT;
            public static final double RESISTANCE = 12 / STALL_CURRENT;
            public static final double MOTOR_KV = ((FREE_SPEED_RPM / 60) * Math.PI * 2)
                    / (12 - RESISTANCE * FREE_CURRENT);
        }

    }

}

