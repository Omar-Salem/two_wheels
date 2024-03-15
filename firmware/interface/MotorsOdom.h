//
// Created by omar on 2/26/24.
//

#ifndef TWO_WHEELS_MOTORSODOM_H
#define TWO_WHEELS_MOTORSODOM_H

class MotorsOdom {
public:
    MotorsOdom(double v1,
               double p1,
               double v2,
               double p2) : v1(v1),
                            v2(v2),
                            p1(p1),
                            p2(p2) {}

    double v1;
    double v2;
    double p1;
    double p2;
};

#endif //TWO_WHEELS_MOTORSODOM_H
