package org.firstinspires.ftc.teamcode.RobotClasses.util;

import java.sql.SQLOutput;
import java.util.Arrays;

/**
 * We needed a way to rotate a set of points on a cartesian plane
 * without changing the values of those points. This is used in the
 * odometry.
 */


/**
 * <a href = "https://github.com/Centennial-FTC-Robotics/Virus2019-2020/blob/master/TeamCode/src/main/java/org/virus/util/Vector2D.java">
 *     literally just this doc.
 *     </a>
 *     <hr>
 *         This Vector2d can accept deltas rather than concrete points.
 */
public class VirusVector2D {

    private String name = "Vector";
    private double R;
    private double theta;
    private double[] components;

    public VirusVector2D(double comp1, double comp2) {

        components = new double[] {comp1, comp2};
        genAngles();
    }

    public VirusVector2D(double newTheta, double newMag, boolean isRad) {

        R = newMag;
        theta = newTheta;

        if (!isRad) {
            theta = Math.toRadians(theta);
        }

        genComp();
    }

    public VirusVector2D(VirusVector2D prevV) {

        components = prevV.getComponents();
        R = prevV.getMag();
        theta = prevV.getTheta();
    }

    //---------- Vector Refresh ----------//

    public void genComp() {
        components = new double[2];

        components[0] = R * Math.cos(theta);
        components[1] = R * Math.sin(theta);
    }

    public void genMag() {

        R = Math.sqrt(Math.pow(components[0], 2) + Math.pow(components[1], 2));
    }

    public void genAngles() {
        genMag();

        if (components[0] == 0 && components[1] == 0) {
            theta = 0;
        } else {

            if (components[0] == 0) {

                theta = Math.PI / 2;

                if (components[1] < 0) {
                    theta *= -1;
                }
            } else {

                theta = Math.atan(components[1] / components[0]);

                if (components[0] < 0) {

                    if (components[1] >= 0) {

                        theta += Math.PI;
                    } else if (components[1] < 0) {
                        theta -= Math.PI;
                    } else {

                        theta = Math.PI;
                    }
                }
            }
        }
    }

    public void setName(String newName) {

        name = newName;
    }

    //---------- Vector Properties ----------//

    public double getMag() {

        return R;
    }

    public double getTheta() {

        return theta;
    }

    public double[] getComponents() {

        return Arrays.copyOf(components, components.length);
    }

    public Double getComponent(int component) {

        if (component >= 0 && component < 2) {

            return components[component];
        }

        return null;
    }

    public String getName() {

        return name;
    }

    //---------- Vector Operations ----------//

    public void setComponents(double[] newComp) {

        if (newComp.length == 2) {

            components = Arrays.copyOf(newComp, newComp.length);
            genAngles();
        }
    }

    public void add(VirusVector2D term_two) {

        double[] two_comp = term_two.getComponents();

        components[0] += two_comp[0];
        components[1] += two_comp[1];

        genAngles();
    }

    public void sub(VirusVector2D term_two) {

        add(invert(term_two));
    }

    public static VirusVector2D add(VirusVector2D term_one, VirusVector2D term_two) {

        double[] one_comp = term_one.getComponents();
        double[] two_comp = term_two.getComponents();

        double newX = one_comp[0] + two_comp[0];
        double newY = one_comp[1] + two_comp[1];

        return (new VirusVector2D(newX, newY));
    }

    public static VirusVector2D sub(VirusVector2D term_one, VirusVector2D term_two) {

        return VirusVector2D.add(term_one, invert(term_two));
    }

    public static VirusVector2D invert(VirusVector2D term_two) {

        VirusVector2D iTwo = new VirusVector2D(term_two);
        iTwo.scale(-1);

        return iTwo;
    }

    public static VirusVector2D scale(VirusVector2D term_one, double newScalar) {

        double angle = term_one.getTheta();
        double scalar = term_one.getMag();

        return new VirusVector2D(angle, scalar * newScalar, true);
    }

    public void zero() {

        this.scale(0);
    }

    public void scale(double scalar) {

        components[0] *= scalar;
        components[1] *= scalar;
    }

    public double dot(VirusVector2D term_two) {

        double[] two_comp = term_two.getComponents();

        return ((components[0] * two_comp[0]) + (components[1] * two_comp[1]));
    }

    public double angleBetween(VirusVector2D v2) {

        double dotProduct = this.dot(v2);
        double magnitudeProducts = R * v2.getMag();

        return Math.acos(dotProduct / magnitudeProducts);
    }

    public static double standardPosAngle(VirusVector2D v) {

        VirusVector2D i = new VirusVector2D(1d, 0d);
        VirusVector2D j = new VirusVector2D(0d, 1d);

        double iAngle = v.angleBetween(i);

        if (v.angleBetween(j) > (Math.PI / 2.0)) {

            iAngle = (Math.PI * 2.0) - iAngle;
        }

        return iAngle;
    }

    /**
     * reverses specified component of the vector
     * @param dimension
     */
    public void flipDimension(int dimension) {

        components[dimension] *= -1;
        genAngles();
    }

    /**
     * This function rotates the vector x radians counterclockwise
     * @param radians
     */
    public void rotate(double radians) {

        theta = (theta + radians) % (2 * Math.PI);
        genComp();
    }
    public String toStringWithoutWeirdBracketThingsSoThatTheLoggerCanWork(){
        String vector = "";
        vector = components[0]+","+components[1];
        return vector;
    }
    public String toString() {

        String vector = name + ": <";
        vector += components[0] + ", ";
        vector += components[1] + ">";

        return vector;
    }

    public boolean equals(VirusVector2D compare) {

        for (int c = 0; c < components.length; c++) {

            if (compare.getComponent(c) != this.getComponent(c)) {

                return false;
            }
        }

        return true;
    }

    public static VirusVector2D[] copy(VirusVector2D[] original) {

        VirusVector2D[] newList = new VirusVector2D[original.length];

        for (int v = 0; v < original.length; v++) {

            newList[v] = new VirusVector2D(original[v]);
        }

        return newList;
    }

    public static void main(String[] args) {

//        for (int angle = 0; angle < 360; angle++) {
//
//            float theta = (float) Math.toRadians(angle);
//            Vector2D v = new Vector2D(theta, 1, true);
//            System.out.println("Angle: " + angle + " genAngle: " + Math.toDegrees(v.getTheta()) + " standardPosAngle: " + Math.toDegrees(standardPosAngle(v)));
//            v.rotate(3 * Math.PI / 4);
//            v.genAngles();
//            System.out.println("Rotated Angle: " + Math.toDegrees(v.getTheta()) + " standardPosAngle: " + Math.toDegrees(standardPosAngle(v)) + "\n");
//
//            if (Math.round(Math.toDegrees(standardPosAngle(v))) != ((angle + 135) % 360)) {
//                System.out.println("BAD");
//                break;
//            }
//        }

        VirusVector2D rightStick = new VirusVector2D(-1, 0);

        for (double a = 130; a <= 140; a += 0.05) {

            VirusVector2D leftStick = new VirusVector2D(0, 1);
            leftStick.rotate(Math.toRadians(a));
            System.out.println(leftStick);
        }
    }
}
