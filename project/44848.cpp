/// Любомир Коев, спец. Информатика ф.н. 44848, 24.06.2014
/// GPL v3.0

#include "Mecho.h"
#include "Lemniscate.h"

// length of vector
double vlen(vect && v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// compares two doubles
bool d_eq(double a, double b)
{
    return fabs(a - b) < 0.001;
}

/**
 * Attach two beams so their 'other' points are connected
 * last parameter is used to specify at which point should they connect
 *
 * @param left - first beam
 * @param right - second beam
 * @return number of points that could be used to connect the beams
 */
int attach_beams(Beam & left, Beam & right, bool use_pos = true)
{
    double r1 = left.getSize(), r2 = right.getSize(),
           dist = vlen(left.getCenter() - right.getCenter());

    // check if beams cant connect - too far apart or too close
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

    // if there is only 1 intersection point - only 1 point to touch
    if(d_eq(dist, r1 + r2)) {
        return 1;
    } else {
        return 2;
    }
}

class cissoid_cls: public MechoDev {
    bool can_flip, // used in animation to show if in cusp or not
         inner; // falg to specify if there is a cusp

    double long_m, short_m; // lengths of 'short' and 'long' beams
    double size, radius, offset, dir, height;

    vect center;
    Pencil pen;

    Beam rotator,
         pivot_up, pivot_down, // short
         drawer_up, drawer_down, // short
         control_up, control_down; // long

public:
    cissoid_cls(const vect &, double = 2.9, double = 5);
    void setTime(double);
    vect getCenter() const { return center; }
private:
    void fixHeights(); // fix heights of all parts
};

cissoid_cls::cissoid_cls(const vect & center, double short_m, double long_m):
    offset(180), dir(1), can_flip(false), long_m(long_m), short_m(short_m),
    inner(long_m / short_m < 2.22), size(short_m), pen(3), height(3),
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
        // if beams cant be connected change direction
        dir *= -1;
        can_flip = true;
        return setTime(time); // call again to repostion correctly
    }

    // passing trough the center of the cusp sets the flag
    if(fabs(offset - 180) < 5) {
        can_flip = true;
    }

    // the rhombus is a line so we are not in the cusp
    if(points == 1 && can_flip) {
        can_flip = false;
        inner = !inner;
    }

    pen.setCenter(drawer_up.otherPoint().setZ(0));
    fixHeights();
}

void cissoid_cls::fixHeights()
{
    rotator.setZ(height + 1.5);

    control_down.setZ(height + 1);
    control_up.setZ(height + 1);

    pivot_down.setZ(height + .5);
    pivot_up.setZ(height + .5);

    drawer_down.setZ(height);
    drawer_up.setZ(height);
}


cissoid_cls * cs;
Button2D * btn_change_short, * btn_change_speed;
int short_beam = 10, long_beam = 16;

/**
 * Toggle short beam length
 */
void toggle_short()
{
    vect center = cs->getCenter();
    delete cs;
    static int cycle = 0;
    cycle = (cycle + 1) % 4;
    cs = new cissoid_cls(center, short_beam - cycle, long_beam);
    btn_change_short->setState(cycle);
}

/**
 * Toggle speed of animation
 */
void toggle_speed()
{
    static int cycle = 0;
    cycle = (cycle + 1) % 4;
    cs->setSpeed(cs->getSpeed() + 50 * (2 - cycle));
    btn_change_speed->setState(cycle);
}



int main()
{
    initMecho("Cissoid", BUTTON_EXIT);
    viewDistance = 100;

    cs = new cissoid_cls({0, 0, 0, 0}, short_beam, long_beam);

    btn_change_short = new Button2D("toggle", GLFW_KEY_SPACE, toggle_short, 0, 4);
    btn_change_speed = new Button2D("time", GLFW_KEY_SPACE, toggle_speed, 0, 4);

    while(runningMecho()) {
        cs->setTime(t);
    }

    finitMecho();
    return 0;
}

