#include "main.h"

void reset();

void setDrive(int left, int right);

void setArm(int power);

void drivemotors(bool state = false);

void liftArm();

std::string move(int units, int volt);

std::string turn(double angle, int volt);

std::string sTurn(const double *x, double y, int volt);
