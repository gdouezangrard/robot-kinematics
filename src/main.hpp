struct position {
    double angle1_J1;
    double angle2_J1;
    double angle3_J1;

    double angle1_J2;
    double angle2_J2;
    double angle3_J2;

    double angle1_J3;
    double angle2_J3;
    double angle3_J3;

    double angle1_J4;
    double angle2_J4;
    double angle3_J4;

    double t;
};

struct angles {
    double theta0;
    double theta1;
    double theta2;
};

enum legs {LEG1, LEG2, LEG3, LEG4};

static double l0 = 0.046;
static double l1 = 0.065;
static double l2 = 0.085;

static double l = l0 + l1 + l2;

enum etats {
    COUCHE,
    DEBOUT,
    STEP1,
    STEP2,
    MARCHER,
    ROTATION1,
    ROTATION2,
    ROTATION3,
    ROTATION4
};

enum art {
    ART1_J1,
    ART2_J1,
    ART3_J1,
    ART1_J2,
    ART2_J2,
    ART3_J2,
    ART1_J3,
    ART2_J3,
    ART3_J3,
    ART1_J4,
    ART2_J4,
    ART3_J4
};
