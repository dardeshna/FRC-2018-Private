package com.palyrobotics.frc2018.config.fields;

import com.palyrobotics.frc2018.config.Constants;

public class FieldSelector {
    public static void configureFieldMeasurements() {
        switch(Constants.kFieldName) {
            case TEAM_8:
                Team8Field.configureFieldMeasurements();
                break;
            case TEAM_254:
                Team254Field.configureFieldMeasurements();
                break;
            case AZN_PRACTICE:
                AZNPracticeField.configureFieldMeasurements();
                break;
            case AZN:
                AZNField.configureFieldMeasurements();
                break;
            case SVR_PRACTICE:
                SVRPracticeField.configureFieldMeasurements();
                break;
            case SVR:
                SVRField.configureFieldMeasurements();
                break;
            case CMP_PRACTICE:
                CMPPracticeField.configureFieldMeasurements();
                break;
            case CMP:
                CMPField.configureFieldMeasurements();
                break;
        }
    }
}
