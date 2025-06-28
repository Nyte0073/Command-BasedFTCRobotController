package org.firstinspires.ftc.teamcode.TestingFolder;

public class InWorkJavaClass {

    volatile boolean completeRotateNeeded,
    wheelsNeedToBeReset, driveNeeded,

    driveActive, completeRotateActive, resetWheelActive;

    Runnable[] runnables = new Runnable[5];

    final Object driveLock = new Object();

    Thread[] threads = {
            new Thread( //Driving thread.
                    () -> {
                       while(!Thread.currentThread().isInterrupted()) {
                           synchronized (driveLock) {
                               while(completeRotateNeeded) {
                                   try {
                                       driveLock.wait();
                                   } catch(Exception e) {
                                       Thread.currentThread().interrupt();
                                       break;
                                   }
                               }

                                if(driveNeeded) { //Both the methods below need to be constantly updated by the program.
                                    runnables[0].run(); //Running the applyFieldOrientedSwerve/applyRobotOrientedSwerve methods.
                                    runnables[1].run(); //Running the setPower methods.
                                }

                                if(!completeRotateNeeded) {
                                    driveLock.notifyAll();
                                }

                                driveActive = false;
                           }

                           try {
                               Thread.sleep(25);
                           } catch(Exception e) {
                               Thread.currentThread().interrupt();
                               break;
                           }
                       }
                    }
            ),

            new Thread( //Complete rotate thread.
                    () -> {
                        while(!Thread.currentThread().isInterrupted()) {
                            if(completeRotateNeeded) { //Both those methods below need to be constantly updated by the program.

                                    runnables[2].run(); //Runs the completeRotate method.
                                    runnables[3].run(); //Runs the setPowerForCompleteRotate method.
                                    completeRotateNeeded = false;
                                    completeRotateActive = false;

                                    try {
                                        Thread.sleep(20);
                                    } catch(Exception e) {
                                        Thread.currentThread().interrupt();
                                    }
                            }
                        }
                    }
            ),

            new Thread( //Reset wheel thread.
                    () -> {
                        while(!Thread.currentThread().isInterrupted()) {
                            synchronized (driveLock) {

                                if(wheelsNeedToBeReset) {

                                    /*For tis specific runnable below, you need to make sure that the code for the resetWheelHeading()
                                    * method inside this runnable is updated WITHIN THE THREAD ITSELF. It is because these values can change
                                    * very quickly and you need them to be EXACTLY up to date for the code in this runnable to work
                                    * effectively.*/

                                    runnables[4].run(); //This method needs to be constantly updated by the program.
                                    resetWheelActive = false;
                                }
                            }

                            try {
                                Thread.sleep(20);
                            } catch(Exception e) {
                                Thread.currentThread().interrupt();
                                break;
                            }
                        }
                    }
            )
    };
}