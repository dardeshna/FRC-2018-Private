package com.palyrobotics.frc2018.util.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
