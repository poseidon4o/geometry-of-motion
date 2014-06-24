#include "Mecho.h"
#include "Lemniscate.h"

double vlen(vect && v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

bool d_eq(double a, double b)
{
    return fabs(a - b) < 0.001;
}

int attach_beams(Beam & left, Beam & right, bool use_pos = true)
{
    double r1 = left.getSize(), r2 = right.getSize(),
           dist = vlen(left.getCenter() - right.getCenter());

    if( (dist > r1 + r2) ||
        (dist < fabs(r1 - r2)) ||
        ((dist == 0) && d_eq(r1, r2))
       ) {
        left.setH(0);
        right.setH(0);
        return 0;
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

    if(d_eq(dist, r1 + r2)) {
        return 1;
    } else {
        return 2;
    }
}

class cissoid_cls: public MechoDev {
    bool can_flip, inner;

    double long_m, short_m;
    double size, radius, offset, dir;
    vect center;
    Pencil pen;

    Beam rotator,
         pivot_up, pivot_down, // short
         drawer_up, drawer_down, // short
         control_up, control_down; // long

public:
    cissoid_cls(const vect &, double = 2.9, double = 5);
    void setTime(double);
private:
    void fixHeights();
};

cissoid_cls::cissoid_cls(const vect & center, double short_m, double long_m):
    offset(180), dir(1), can_flip(false), long_m(long_m), short_m(short_m),
    inner(long_m / short_m < 2.22), size(short_m), pen(0),
    center(center), rotator(short_m),
    pivot_up(short_m), pivot_down(short_m),
    drawer_up(short_m), drawer_down(short_m),
    control_up(long_m), control_down(long_m)
{
    setSpeed(50);
    rotator.setCenter(center);

    control_up.setCenter(rotator.otherPoint());
    control_down.setCenter(rotator.otherPoint());

    pivot_up.setCenter({center.x + size, center.y, center.z, center.w});
    pivot_down.setCenter({center.x + size, center.y, center.z, center.w});

    drawer_up.setCenter(control_up.otherPoint());
    drawer_down.setCenter(control_down.otherPoint());

    setTime(0);
    pen.penDown();
}


void cissoid_cls::setTime(double time)
{
    offset += dt * getSpeed() * dir;
    rotator.setH(offset);

    control_up.setCenter(rotator.otherPoint());
    control_down.setCenter(rotator.otherPoint());

    attach_beams(control_up, pivot_up, false);
    attach_beams(control_down, pivot_down, true);

    drawer_up.setCenter(pivot_up.otherPoint());
    drawer_down.setCenter(pivot_down.otherPoint());

    int points = attach_beams(drawer_down, drawer_up, !inner);
    if(!points) {
        dir *= -1;
        can_flip = true;
        return setTime(time);
    }

    if(fabs(offset - 180) < 5) {
        can_flip = true;
    }

    if(points == 1 && can_flip) {
        can_flip = false;
        inner = !inner;
    }

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
    cissoid_cls cs({0, 0, 0, 0}, 10, 16);

    while(runningMecho()) {
        cs.setTime(t);
    }

    finitMecho();
    return 0;
}

