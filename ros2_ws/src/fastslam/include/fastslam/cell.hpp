#ifndef CELL_HPP
#define CELL_HPP

#include <utility>

namespace fastslam
{
    class Cell {
        public:

            explicit Cell() : acc_x_(0.0), acc_y_(0.0), n_(0) {}

            std::pair<double,double> getMean() const; 

            bool hasBeenHit()  const; 

            bool isOccupied(float occupied_threshold) const;

            void updateHit(double x, double y); 

        private:
            double acc_x_, acc_y_;
            int n_;

    }; 

}

#endif