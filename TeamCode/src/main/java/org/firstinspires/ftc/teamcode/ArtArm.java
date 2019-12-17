/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

/**
 * This a class that does calculations and ultimately returns the counts/degrees of the base motors
 * and end effector servo joint
 */


public class ArtArm {
    private double l1, l2; //lengths of arm segments
    private double g1, g2; //chain gears
    private double gb, gm; // big gear and motor gear
    private double x, y;

    ArtArm(double length1, double length2, double baseGear, double motorGear, double gear1, double gear2) {

        l1 = length1;
        l2 = length2;

        gb = baseGear;
        gm = motorGear;

        g1 = gear1;
        g2 = gear2;

        x = 2;
        y = 1;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    private double getDist(){
        //System.out.print("getDist: ");
        //System.out.println(Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2)));
        return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2));
    }

    public double maxDist(){
        //System.out.print("maxDist: ");
        //System.out.println(l1 + l2);
        return l1 + l2;
    }

    public double maxY(){
        //System.out.print("maxY: ");
        //System.out.println(Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getX(), 2)));
        return Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getX(), 2));
    }

    public double maxX(){
        //System.out.print("maxX: ");
        //System.out.println(Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getY(), 2)));
        return Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getY(), 2));
    }

    private double getD1(){
        double d1 = Math.acos((Math.pow(getDist(), 2) + Math.pow(l1, 2) - Math.pow(l2, 2)) / (2*getDist()*l1));
        //System.out.print("D1: ");
        //System.out.println(d1);
        return d1;
    }

    private double getD2(){
        //System.out.print("D2: ");
        double d2;
        if(getX() == 0)
            d2 = Math.PI/2.0;
        else
            d2 = Math.atan2(getY(), getX());
        //System.out.println(d2);
        return d2;
    }

    public double getA1(){
        //(d1 + d2)
        double a1 = getD1() + getD2();
        System.out.print("A1: ");
        System.out.println(a1 * 180/Math.PI);
        return a1;
    }

    public double getA2(){
        double a2 = Math.acos((-1 * Math.pow(getDist(), 2) + Math.pow(l1, 2) + Math.pow(l2, 2)) / (2*l1*l2));
        System.out.print("A2: ");
        System.out.println(a2 * 180/Math.PI);
        if(a2 > Math.PI)
            a2 = 2 * Math.PI - a2;
        return a2;
    }

    public double getA3(){
        double a3 = (g2/g1)*getA2() + getA1();
        System.out.print("A3: ");
        System.out.println(a3 * 180/Math.PI);
        return a3;
    }


    public int getM1(){
        int m1 = (int)((getA1()/* - 120*1680/360.0*/)*(gb/gm) * 1120.0 / (2 * Math.PI)) - 1365;
        //System.out.print("M1: ");
        //System.out.println(m1);
        return m1; //motor counts
    }

    public int getM2(){
        int m2 = ((int)((getA3()/* - 33.5*1680/360.0*/)*(gb/gm) * 1120.0 / (2 * Math.PI))) - 1532;
        //System.out.print("M2: ");
        //System.out.println(m2);
        return m2; //motor counts
    }

    public double getEERad(){
        double rad = ((Math.PI - getD2()) + (Math.PI - getD1() - getA2()));
        //System.out.print("EERad: ");
        //System.out.println(rad);
        return rad;
    }

    public double[] goToXY(double xcor, double ycor){
        x = xcor;
        y = ycor;

        double[] array = {getM1(), getM2(), getEERad()};
        System.out.print("goTo: ");
        System.out.println("[" + array[0] + " " + array[1] + " " + array[2] + "]");
        return array;
    }

}
