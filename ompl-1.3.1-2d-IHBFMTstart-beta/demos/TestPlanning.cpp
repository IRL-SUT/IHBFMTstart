#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/IHBFMT.h> //Our IHBFMT
#include <ompl/util/PPM.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const char *ppm_file)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            auto space(std::make_shared<ob::RealVectorStateSpace>());
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            std::cout << "maxWidth_: " << maxWidth_ << std::endl;
            std::cout << "maxHeight_: " << maxHeight_ << std::endl;
            ss_ = std::make_shared<og::SimpleSetup>(space);

            // set state validity checking for this space
            ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            ss_->setPlanner(std::make_shared<og::IHBFMT>(ss_->getSpaceInformation()));

            
            auto ihbfmt = std::make_shared<ompl::geometric::IHBFMT>(ss_->getSpaceInformation());

            ihbfmt->params().setParam("optimality", "true");
            ihbfmt->params().setParam("cost_threshold", "1750");

            ss_->setPlanner(ihbfmt);

        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);

        auto opt = ss_->getProblemDefinition()->getOptimizationObjective();
        ss_->print();
        // generate a few solutions; all will be added to the goal;
        for (int i = 0 ; i < 1 ; ++i)
        {
            if (ss_->getPlanner())
                ss_->getPlanner()->clear();
            ss_->solve(200); // Set Planning Time
        }
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath())
        {
            ss_->simplifySolution();
            og::PathGeometric &p = ss_->getSolutionPath();
            //            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);
            OMPL_DEBUG("Simple path cost: %f", ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective()));
            return true;
        }
        
        return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);
        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;


};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str());

    if (env.plan(50, 50, 777, 1265))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
