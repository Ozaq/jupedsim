//
// Created by Tobias Schrödter on 15.10.18.
//

#include "WaitingArea.h"

int WaitingArea::getMaxNumPed() const
{
     return maxNumPed;
}

void WaitingArea::setMaxNumPed(int maxNumPed)
{
     WaitingArea::maxNumPed = maxNumPed;
}

int WaitingArea::getMinNumPed() const
{
     return minNumPed;
}

void WaitingArea::setMinNumPed(int minNumPed)
{
     WaitingArea::minNumPed = minNumPed;
}

bool WaitingArea::isOpen() const
{
     return open;
}

void WaitingArea::setOpen(bool open)
{
     WaitingArea::open = open;
}

const std::map<int, double>& WaitingArea::getNextGoals() const
{
     return nextGoals;
}

bool WaitingArea::setNextGoals(const std::map<int, double>& nextGoals)
{
     WaitingArea::nextGoals = nextGoals;
     return checkProbabilities();
}

bool WaitingArea::checkProbabilities()
{
     double p =0.;
     for ( std::map<int, double>::iterator it = nextGoals.begin(); it != nextGoals.end(); it++ ){
          p += it->second;
     }

     return (p>0.9999) && (p<1.000001);
}

void WaitingArea::updateProbabilities()
{

}

std::string WaitingArea::toString()
{
     std::string out;
     char buffer [50];


     out.append("WaitingArea[\n");
     sprintf(buffer, "\tid=%d\n", _id);
     out.append(buffer);
     sprintf(buffer, "\tcaption=%s\n", _caption.c_str());
     out.append(buffer);
     sprintf(buffer, "\tmin_peds=%d\n", minNumPed);
     out.append(buffer);
     sprintf(buffer, "\tmax_peds=%d\n", maxNumPed);
     out.append(buffer);
     sprintf(buffer, "\tis_open=%d\n", open);
     out.append(buffer);
     sprintf(buffer, "\twaiting_time=%f\n", waitingTime);
     out.append(buffer);

     for (auto const& nextWa : nextGoals){
          sprintf(buffer, "\tnext id=%d\tp=%f\n", nextWa.first, nextWa.second);
          out.append(buffer);
     }
     out.append("]");
     return out;

}

double WaitingArea::getWaitingTime() const
{
     return waitingTime;
}

void WaitingArea::setWaitingTime(double waitingTime)
{
     WaitingArea::waitingTime = waitingTime;
}

std::vector<Line> WaitingArea::getAllLines()
{
}

int WaitingArea::GetNextGoal()
{
     double random = std::rand()/RAND_MAX;
     double cumProb = 0.;

     for (auto& nextGoal : nextGoals){
          cumProb += nextGoal.second;
          if (random <= cumProb ){
               return nextGoal.first;
          }
     }

}

void WaitingArea::addPed()
{
     numPed++;
}

void WaitingArea::removePed()
{
     numPed--;
}

void WaitingArea::startTimer(double time)
{
     startTime = time;
     std::cout << "Timer started at " << startTime << std::endl;
}

bool WaitingArea::isWaiting(double time)
{
     if ((numPed >= minNumPed) && (startTime < 0. )){
          startTimer(time);
     }

     if ((time < startTime + waitingTime) || (startTime < 0. )){
          std::cout << "Waiting ..." << std::endl;
          return true;
     }else{
          std::cout << "Waiting over!" << std::endl;
          return false;
     }
}

int WaitingArea::getNumPed()
{
     return numPed;
}
