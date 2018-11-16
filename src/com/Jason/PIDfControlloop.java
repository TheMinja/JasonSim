package com.Jason;

import java.util.concurrent.Callable;

public class PIDfControlloop {
    private double p;
    private double i;
    private double d;
    private double f;

    private double output;
    private double maxOutput;
    private double goal;
    private double error;
    private double accumulatedError;
    private double errorVelocity;
    private double lastError;
    private long intervalInMillis;
    private Thread periodic;
    private double position;
    private Double maxIContribution = null;
    private Callable<Boolean> fCondition;

    /**
     * @param p
     * @param i
     * @param d
     * @param f if a different input of error is desired, pass in null for talon
     * @param intervalInMillis
     * @param maxOutput goal is in units of encoder tics if using a talon
     */
    public PIDfControlloop(double p, double i, double d, double f, long intervalInMillis, double maxOutput) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.maxOutput = maxOutput;
        if (intervalInMillis <= 0)
            intervalInMillis = 1;
        this.intervalInMillis = intervalInMillis;
        periodic = new Thread(new Runnable() {
            public void run() {
                while (true) {
                    try {
                        pidCycle();
                        Thread.sleep(PIDfControlloop.this.intervalInMillis);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                }
            }
        });

    }
    double interval = 0;
    long lastTime = System.currentTimeMillis();

    public void pidCycle() throws Exception {
        error = goal - position;
        interval = intervalInMillis / 1000d;

        output = 0;

//		if(fCondition == null || fCondition.call()){
//			output += f;
//		}

        output += p * error;

        accumulatedError += error * interval;
        if (maxIContribution == null)
            output += i * accumulatedError;
        else {
            if (accumulatedError > 0) {
                output += Math.min(i * accumulatedError, maxIContribution);
            }else{
                output += Math.max(i * accumulatedError, -maxIContribution);
            }

        }
        errorVelocity = ((error - lastError) / (interval));
        output += d * errorVelocity;

        lastError = error;
        lastTime = System.currentTimeMillis();

        if (Math.abs(output) >= maxOutput) {
            if (output > 0) {
                output = maxOutput;
            } else {
                output = -maxOutput;
            }
        }
    }

    public double getOutput() {
        return output;
    }

    public void setNewPosition(double position) {
        this.position = position;
    }

    public void startPID(double goal) {
        this.goal = goal;
        if (!periodic.isAlive())
            periodic.start();
    }

    public void stopPID() {
        periodic.interrupt();
        lastError = 0;
        accumulatedError = 0;
        errorVelocity = 0;
    }

    public void updateSetPoint(double newGoal) {
        this.goal = newGoal;
    }

    public double getError() {
        return error;
    }

    public void configMaxIContribution(double maxContribution) {
        this.maxIContribution = maxContribution;
    }

    public void resetAccumulatedError(){
        this.accumulatedError = 0;
    }

    public double getAccumulatedError(){return accumulatedError;}

    /**
     * The F gain will only be applied if this condition is true, if not specified, F gain will always be used.
     */
    public void useFCondition(Callable<Boolean> fCondition){
        this.fCondition = fCondition;
    }

    public void setF(double f){
        this.f = f;
    }

    public void setP(double p){
        this.p = p;
    }

    public void setI(double i){
        this.p = i;
    }

    public void setD(double d){
        this.d = d;
    }
}
