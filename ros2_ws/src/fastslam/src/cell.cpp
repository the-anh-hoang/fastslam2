#include "fastslam/cell.hpp"
#include <cassert>

namespace fastslam 
{
    std::pair<double, double> Cell::getMean() const{
        assert(n_ > 0);
        return {acc_x_/n_,acc_y_/n_};
    }

    bool Cell::isOccupied(float min_hits) const {
        return n_ >= min_hits; 
    }

    void Cell::updateHit(double x, double y) {
        acc_x_ += x;
        acc_y_ += y;
        n_++;
    }
}