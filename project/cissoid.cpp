#include "Mecho.h"
#include "Lemniscate.h"

double vlen(vect && v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void attach_beams(Beam & left, Beam & right, bool use_pos = true)
{
    double r1 = left.getSize(), r2 = right.getSize(),
           dist = vlen(left.getCenter() - right.getCenter());

    if( (dist > r1 + r2) ||
        (dist < fabs(r1 - r2)) ||
        ((dist == 0) && (r1 == r2))
       ) {
        left.setH(0);
        right.setH(0);
        return;
    }

    double a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
    double h = sqrt(r1 * r1 - a * a);

    double cx2 = left.getCenter().x + a * (right.getCenter().x - left.getCenter().x) / dist;
    double cy2 = left.getCenter().y + a * (right.getCenter().y - left.getCenter().y) / dist;

    vect pt1 = { (cx2 + h * (right.getCenter().y - left.getCenter().y) / dist), (cy2 - h * (right.getCenter().x - left.getCenter().x) / dist), 0, 0};
    vect pt2 = { (cx2 - h * (right.getCenter().y - left.getCenter().y) / dist), (cy2 + h * (right.getCenter().x - left.getCenter().x) / dist), 0, 0};

    if(use_pos) {
        left.setH(atan2(pt1.y - left.getCenter().y, pt1.x - left.getCenter().x) * 180 / M_PI);
        right.setH(atan2(pt1.y - right.getCenter().y, pt1.x - right.getCenter().x) * 180 / M_PI);
    } else {
        left.setH(atan2(pt2.y - left.getCenter().y, pt2.x - left.getCenter().x) * 180 / M_PI);
        right.setH(atan2(pt2.y - right.getCenter().y, pt2.x - right.getCenter().x) * 180 / M_PI);
    }

}

class cissoid_cls: public MechoDev {
    const int mult1 = 2.9, mult2 = 3.5;
    double size, radius, offset, dir, stop_angle;
    vect center;
    Pencil pen;

    Beam rotator,
         pivot_up, pivot_down, // short
         drawer_up, drawer_down, // short
         control_up, control_down; // long

public:
    cissoid_cls(double, const vect &);
    void setTime(double);
private:
    void fixHeights();
};

cissoid_cls::cissoid_cls(double size, const vect & center):
    offset(0), dir(1), stop_angle(55),
    size(size), pen(size), rotator(size), center(center),
    pivot_up(size * mult1), pivot_down(size * mult1),
    drawer_up(size * mult1), drawer_down(size * mult1),
    control_up(size * mult2), control_down(size * mult2)
{
    setSpeed(50);
    rotator.setCenter(center);

    control_up.setCenter(rotator.otherPoint());
    control_down.setCenter(rotator.otherPoint());

    pivot_up.setCenter({center.x + size, center.y, center.z, center.w});
    pivot_down.setCenter({center.x + size, center.y, center.z, center.w});

    drawer_up.setCenter(control_up.otherPoint());
    drawer_down.setCenter(control_down.otherPoint());

    offset = stop_angle;
    setTime(0);

    pen.penDown();
    fixHeights();
}


void cissoid_cls::setTime(double time)
{
    if(dir == 1 && offset >= 360 - stop_angle) {
        dir = -1;
    } else if(dir == -1 && offset <= stop_angle) {
        dir = 1;
    }

    offset += dt * getSpeed() * dir;
    rotator.setH(offset);

    control_up.setCenter(rotator.otherPoint());
    control_down.setCenter(rotator.otherPoint());

    attach_beams(control_up, pivot_up, false);
    attach_beams(control_down, pivot_down, true);

    drawer_up.setCenter(pivot_up.otherPoint());
    drawer_down.setCenter(pivot_down.otherPoint());

    attach_beams(drawer_down, drawer_up, true);

    fixHeights();
    pen.setCenter(drawer_up.otherPoint().setZ(0));
}

void cissoid_cls::fixHeights()
{
    rotator.setZ(3);

    control_down.setZ(2);
    control_up.setZ(2);

    pivot_down.setZ(1);
    pivot_up.setZ(1);

    drawer_down.setZ(0);
    drawer_up.setZ(0);
}

int main()
{
    initMecho("Cissoid", BUTTON_EXIT);
    cissoid_cls cs(5, {0, 0, 0, 0});

    while(runningMecho()) {
        cs.setTime(t);
    }

    finitMecho();
    return 0;
}

