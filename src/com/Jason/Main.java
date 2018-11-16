package com.Jason;

import com.sun.xml.internal.ws.developer.SerializationFeature;

import java.io.*;
import java.time.LocalDateTime;

public class Main {

    public static double dt = .01;
    public static double setpoint = 45;
    public static int testTime = 5;
    public static void main(String[] args) throws Exception {
        Main m = new Main();
        m.init();
//        m.testSetpoint(setpoint);
//        m.testSetpoint(setpoint*2);
        m.testSetpoint(90);
        m.newSetpoint(45);
//        m.testMovingSetpoint(90);
    }

    public ArmMechanism arm;
    public void testSetpoint(double setPoint) throws Exception {
        arm = new ArmMechanism(100,
                ArmMechanism.MotorType.PRO,
                2, 2, 3d);
        for (int i = 0; i < testTime*(1/dt); i++) {
            for (int j = 0; j < 10; j++) {
                arm.calculatePosition(1);
                arm.calculateVelocity(1);
            }
            double voltage = calcVoltage(arm.getPosition(), setPoint);
            arm.updateVoltage(voltage);
            writeToMainFile(i * dt, arm.getPosition(), arm.getVelocity(), setPoint, voltage, arm.getAcceleration(), pid.getError(), pid.getAccumulatedError());
        }
    }

    public void newSetpoint(double setPoint) throws Exception {
        for (int i = 0; i < testTime*(1/dt); i++) {
            for (int j = 0; j < 10; j++) {
                arm.calculatePosition(1);
                arm.calculateVelocity(1);
            }
            double voltage = calcVoltage(arm.getPosition(), setPoint);
            arm.updateVoltage(voltage);
            writeToMainFile(5+ i * dt, arm.getPosition(), arm.getVelocity(), setPoint, voltage, arm.getAcceleration(), pid.getError(), pid.getAccumulatedError());
        }
    }


    public void testMovingSetpoint(double setPointVelocity) throws Exception {
        arm = new ArmMechanism(100,
                ArmMechanism.MotorType.PRO,
                2, 2, 3d);
        double setPoint = 0;
        for (int i = 0; i < testTime*(1/dt); i++) {
            for (int j = 0; j < 10; j++) {
                arm.calculatePosition(1);
                arm.calculateVelocity(1);
            }
            double voltage = calcVoltage(arm.getPosition(), setPoint);
            arm.updateVoltage(voltage);
            setPoint += (setPointVelocity / 100d);
            writeToMainFile(i * dt, arm.getPosition(), arm.getVelocity(), setPoint, voltage, arm.getAcceleration(), pid.getError(), pid.getAccumulatedError());
        }
    }

    private PIDfControlloop pid = new PIDfControlloop(.08, 0, 0.05, 0, 10, 1);

    public double calcVoltage(double position, double setPoint) throws Exception {
        pid.updateSetPoint(setPoint);
        pid.setNewPosition(position);
        pid.pidCycle();
        return pid.getOutput() * 12d;
    }

    String pathName = System.getProperty("java.io.tmpdir").concat("/WristSimulation.csv");
    private PrintWriter pw;
    private Long lastId = 0L;

    public void init() throws IOException {
        pw = new PrintWriter(new FileOutputStream(
                new File(pathName),
                false));
        StringBuilder sb = new StringBuilder();
        sb.append("timeStamp");
        sb.append(",");
        sb.append("position");
        sb.append(",");
        sb.append("velocity");
        sb.append(",");
        sb.append("setPoint");
        sb.append(",");
        sb.append("voltage");
        sb.append(",");
        sb.append("acceleration");
        sb.append(",");
        sb.append("error");
        sb.append(",");
        sb.append("accumError");
        sb.append("\n");
        pw.write(sb.toString());
        pw.flush();
    }

    public void writeToMainFile(double... values) throws FileNotFoundException {
        StringBuilder sb = new StringBuilder();
        for (double num : values) {
            sb.append(num + ",");
        }
//        System.out.println(sb.toString());
        sb.append('\n');

        pw.write(sb.toString());
        pw.flush();
    }


}
