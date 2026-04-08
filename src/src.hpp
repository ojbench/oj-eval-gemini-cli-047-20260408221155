#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <vector>
#include <cmath>
#include <algorithm>

struct Line {
    Vec point;
    Vec dir;
};

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;
    int round_count = 0;

    bool linearProgram1(const std::vector<Line>& lines, int lineNo, double radius, const Vec& optVelocity, bool directionOpt, Vec& result) {
        double dotProduct = lines[lineNo].point.dot(lines[lineNo].dir);
        double discriminant = dotProduct * dotProduct + radius * radius - lines[lineNo].point.norm_sqr();

        if (discriminant < 0.0) {
            return false;
        }

        double sqrtDiscriminant = std::sqrt(discriminant);
        double tLeft = -dotProduct - sqrtDiscriminant;
        double tRight = -dotProduct + sqrtDiscriminant;

        for (int i = 0; i < lineNo; ++i) {
            double denominator = lines[lineNo].dir.cross(lines[i].dir);
            double numerator = lines[i].dir.cross(lines[lineNo].point - lines[i].point);

            if (std::abs(denominator) <= 1e-7) {
                if (numerator < 0.0) {
                    return false;
                } else {
                    continue;
                }
            }

            double t = numerator / denominator;

            if (denominator >= 0.0) {
                tRight = std::min(tRight, t);
            } else {
                tLeft = std::max(tLeft, t);
            }

            if (tLeft > tRight) {
                return false;
            }
        }

        if (directionOpt) {
            if (optVelocity.dot(lines[lineNo].dir) > 0.0) {
                result = lines[lineNo].point + lines[lineNo].dir * tRight;
            } else {
                result = lines[lineNo].point + lines[lineNo].dir * tLeft;
            }
        } else {
            double t = lines[lineNo].dir.dot(optVelocity - lines[lineNo].point);

            if (t < tLeft) {
                result = lines[lineNo].point + lines[lineNo].dir * tLeft;
            } else if (t > tRight) {
                result = lines[lineNo].point + lines[lineNo].dir * tRight;
            } else {
                result = lines[lineNo].point + lines[lineNo].dir * t;
            }
        }

        return true;
    }

    int linearProgram2(const std::vector<Line>& lines, double radius, const Vec& optVelocity, bool directionOpt, Vec& result) {
        if (directionOpt) {
            result = optVelocity * radius;
        } else if (optVelocity.norm_sqr() > radius * radius) {
            result = optVelocity.normalize() * radius;
        } else {
            result = optVelocity;
        }

        for (int i = 0; i < lines.size(); ++i) {
            if (lines[i].dir.cross(lines[i].point - result) > 0.0) {
                Vec tempResult = result;
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                    result = tempResult;
                    return i;
                }
            }
        }

        return lines.size();
    }

    void linearProgram3(const std::vector<Line>& lines, int numObstLines, int beginLine, double radius, Vec& result) {
        double distance = 0.0;

        for (int i = beginLine; i < lines.size(); ++i) {
            if (lines[i].dir.cross(lines[i].point - result) > distance) {
                std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);

                for (int j = numObstLines; j < i; ++j) {
                    Line line;
                    double determinant = lines[i].dir.cross(lines[j].dir);

                    if (std::abs(determinant) <= 1e-7) {
                        if (lines[i].dir.dot(lines[j].dir) > 0.0) {
                            continue;
                        } else {
                            line.point = (lines[i].point + lines[j].point) * 0.5;
                        }
                    } else {
                        line.point = lines[i].point + lines[i].dir * (lines[j].dir.cross(lines[i].point - lines[j].point) / determinant);
                    }

                    line.dir = (lines[j].dir - lines[i].dir).normalize();
                    projLines.push_back(line);
                }

                Vec tempResult = result;
                if (linearProgram2(projLines, radius, Vec(-lines[i].dir.y, lines[i].dir.x), true, result) < projLines.size()) {
                    result = tempResult;
                }

                distance = lines[i].dir.cross(lines[i].point - result);
            }
        }
    }

    void computeORCALines(const Vec& pos_i, const Vec& v_i, double r_i,
                          const Vec& pos_j, const Vec& v_j, double r_j,
                          double tau, Line& line) {
        Vec relativePosition = pos_j - pos_i;
        Vec relativeVelocity = v_i - v_j;
        double distSq = relativePosition.norm_sqr();
        double combinedRadius = r_i + r_j + 0.02; // Add a small margin
        double combinedRadiusSq = combinedRadius * combinedRadius;

        Vec u;

        if (distSq > combinedRadiusSq) {
            Vec w = relativeVelocity - relativePosition / tau;
            double wLengthSq = w.norm_sqr();
            double dotProduct1 = w.dot(relativePosition);

            if (dotProduct1 < 0.0 && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
                double wLength = std::sqrt(wLengthSq);
                Vec unitW = w / wLength;

                line.dir = Vec(unitW.y, -unitW.x);
                u = unitW * (combinedRadius / tau - wLength);
            } else {
                double leg = std::sqrt(distSq - combinedRadiusSq);

                if (relativePosition.cross(w) > 0.0) {
                    line.dir = Vec(relativePosition.x * leg - relativePosition.y * combinedRadius,
                                   relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                } else {
                    line.dir = -Vec(relativePosition.x * leg + relativePosition.y * combinedRadius,
                                    -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                }

                double dotProduct2 = relativeVelocity.dot(line.dir);
                u = line.dir * dotProduct2 - relativeVelocity;
            }
        } else {
            double invTimeStep = 1.0 / TIME_INTERVAL;
            Vec w = relativeVelocity - relativePosition * invTimeStep;
            double wLength = w.norm();
            Vec unitW = w / wLength;

            line.dir = Vec(unitW.y, -unitW.x);
            u = unitW * (combinedRadius * invTimeStep - wLength);
        }

        line.point = v_i + u * 0.5;
    }

public:

    Vec get_v_next() {
        Vec pref_v = pos_tar - pos_cur;
        double dist = pref_v.norm();
        if (dist > 1e-5) {
            if (dist > v_max * TIME_INTERVAL) {
                pref_v = pref_v.normalize() * v_max;
            } else {
                pref_v = pref_v / TIME_INTERVAL;
            }
            // Add a tiny perturbation to pref_v to break symmetry
            double angle = (id * 137.5 + round_count * 17.3) * PI / 180.0;
            Vec perturbation(std::cos(angle), std::sin(angle));
            pref_v = pref_v + perturbation * 0.01;
        } else {
            pref_v = Vec(0, 0);
        }

        std::vector<Line> lines;
        int num_robots = monitor->get_robot_number();
        for (int i = 0; i < num_robots; ++i) {
            if (i == id) continue;
            Vec pos_j = monitor->get_pos_cur(i);
            Vec v_j = monitor->get_v_cur(i);
            double r_j = monitor->get_r(i);
            
            Line line;
            computeORCALines(pos_cur, v_cur, r, pos_j, v_j, r_j, 2.0, line);
            lines.push_back(line);
        }

        Vec result;
        int failedLine = linearProgram2(lines, v_max, pref_v, false, result);
        if (failedLine < lines.size()) {
            linearProgram3(lines, 0, failedLine, v_max, result);
        }
        
        if (monitor->get_warning()) {
            double angle = (id * 137.5 + round_count * 17.3) * PI / 180.0;
            Vec perturbation(std::cos(angle), std::sin(angle));
            result = result + perturbation * (v_max * 0.5);
            if (result.norm_sqr() > v_max * v_max) {
                result = result.normalize() * v_max;
            }
        }
        
        round_count++;
        return result;
    }
};

#endif //PPCA_SRC_HPP