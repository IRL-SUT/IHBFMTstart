#include <boost/math/constants/constants.hpp>
#include <boost/math/distributions/binomial.hpp>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/fmt/IHBFMT.h>

#include <fstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <chrono>  // time



namespace ompl
{
namespace geometric
{
IHBFMT::IHBFMT(const base::SpaceInformationPtr &si)
    : base::Planner(si, "IHBFMT")
    , freeSpaceVolume_(si_->getStateSpace()->getMeasure())  // An upper bound on the free space volume is the
    // total space volume; the free fraction is estimated
    // in sampleFree
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &IHBFMT::setNumSamples,
                                                    &IHBFMT::getNumSamples, "10:10:1000000");
    ompl::base::Planner::declareParam<double>("radius_multiplier", this, &IHBFMT::setRadiusMultiplier,
                                              &IHBFMT::getRadiusMultiplier, "0.1:0.05:50.");
    ompl::base::Planner::declareParam<bool>("nearest_k", this, &IHBFMT::setNearestK, &IHBFMT::getNearestK, "0,1");
    ompl::base::Planner::declareParam<bool>("balanced", this, &IHBFMT::setExploration, &IHBFMT::getExploration,
                                            "0,1");
    ompl::base::Planner::declareParam<bool>("optimality", this, &IHBFMT::setTermination, &IHBFMT::getTermination,
                                            "0,1");
    ompl::base::Planner::declareParam<bool>("heuristics", this, &IHBFMT::setHeuristics, &IHBFMT::getHeuristics,
                                            "0,1");
    ompl::base::Planner::declareParam<bool>("cache_cc", this, &IHBFMT::setCacheCC, &IHBFMT::getCacheCC, "0,1");
    ompl::base::Planner::declareParam<bool>("extended_fmt", this, &IHBFMT::setExtendedFMT, &IHBFMT::getExtendedFMT,
                                            "0,1");
    ompl::base::Planner::declareParam<double>("cost_threshold", this, &IHBFMT::setCostTheshold,
                                                    &IHBFMT::getCostTheshold, "1:1:1000000");
}

ompl::geometric::IHBFMT::~IHBFMT()
{
    freeMemory();
}

void IHBFMT::setup()
{
    if (pdef_)
    {
        /* Setup the optimization objective. If no optimization objective was
               specified, then default to optimizing path length as computed by the
               distance() function in the state space */
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }
        Open_[0].getComparisonOperator().opt_ = opt_.get();
        Open_[0].getComparisonOperator().heuristics_ = heuristics_;
        Open_[1].getComparisonOperator().opt_ = opt_.get();
        Open_[1].getComparisonOperator().heuristics_ = heuristics_;

        if (!nn_)
            nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<BiDirMotion *>(this));
        nn_->setDistanceFunction([this](const BiDirMotion *a, const BiDirMotion *b)
        {
            return distanceFunction(a, b);
        });

        if (nearestK_ && !nn_->reportsSortedResults())
        {
            OMPL_WARN("%s: NearestNeighbors datastructure does not return sorted solutions. Nearest K strategy "
                      "disabled.",
                      getName().c_str());
            nearestK_ = false;
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void IHBFMT::freeMemory()
{
    if (nn_)
    {
        BiDirMotionPtrs motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            si_->freeState(motion->getState());
            delete motion;
        }
    }
}

void IHBFMT::clear()
{
    Planner::clear();

    connect_motion_=nullptr;

    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    Open_[FWD].clear();
    Open_[REV].clear();
    Open_elements[FWD].clear();
    Open_elements[REV].clear();
    neighborhoods_.clear();
    collisionChecks_ = 0;
}

void IHBFMT::getPlannerData(base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);
    BiDirMotionPtrs motions;
    nn_->list(motions);

    int numStartNodes = 0;
    int numGoalNodes = 0;
    int numEdges = 0;
    int numFwdEdges = 0;
    int numRevEdges = 0;

    int fwd_tree_tag = 1;
    int rev_tree_tag = 2;

    for (auto motion : motions)
    {
        bool inFwdTree = (motion->currentSet_[FWD] != BiDirMotion::SET_UNVISITED);

        // For samples added to the fwd tree, add incoming edges (from fwd tree parent)
        if (inFwdTree)
        {
            if (motion->parent_[FWD] == nullptr)
            {
                // Motion is a forward tree root node
                ++numStartNodes;
            }
            else
            {
                bool success =
                        data.addEdge(base::PlannerDataVertex(motion->parent_[FWD]->getState(), fwd_tree_tag),
                                     base::PlannerDataVertex(motion->getState(), fwd_tree_tag));
                if (success)
                {
                    ++numFwdEdges;
                    ++numEdges;
                }
            }
        }
    }

    // The edges in the goal tree are reversed so that they are in the same direction as start tree
    for (auto motion : motions)
    {
        bool inRevTree = (motion->currentSet_[REV] != BiDirMotion::SET_UNVISITED);

        // For samples added to a tree, add incoming edges (from fwd tree parent)
        if (inRevTree)
        {
            if (motion->parent_[REV] == nullptr)
            {
                // Motion is a reverse tree root node
                ++numGoalNodes;
            }
            else
            {
                bool success =
                        data.addEdge(base::PlannerDataVertex(motion->getState(), rev_tree_tag),
                                     base::PlannerDataVertex(motion->parent_[REV]->getState(), rev_tree_tag));
                if (success)
                {
                    ++numRevEdges;
                    ++numEdges;
                }
            }
        }
    }
}

void IHBFMT::saveNeighborhood(const std::shared_ptr<NearestNeighbors<BiDirMotion *>> &nn, BiDirMotion *m)
{
    // Check if neighborhood has already been saved
    if (neighborhoods_.find(m) == neighborhoods_.end())
    {
        BiDirMotionPtrs neighborhood;
        if (nearestK_)
            nn_->nearestK(m, NNk_, neighborhood);
        else
            nn_->nearestR(m, NNr_, neighborhood);

        if (!neighborhood.empty())
        {
            // Save the neighborhood but skip the first element (m)
            neighborhoods_[m] = std::vector<BiDirMotion *>(neighborhood.size() - 1, nullptr);
            std::copy(neighborhood.begin() + 1, neighborhood.end(), neighborhoods_[m].begin());
        }
        else
        {
            // Save an empty neighborhood
            neighborhoods_[m] = std::vector<BiDirMotion *>(0);
        }
    }
}

void IHBFMT::sampleFree(const std::shared_ptr<NearestNeighbors<BiDirMotion *>> &nn,
                      const base::PlannerTerminationCondition &ptc)
{
    if(!firstSuccessful_)
    {
        std::cout << "sample begin" << std::endl;//test code
        unsigned int nodeCount = 0;
        unsigned int sampleAttempts = 0;
        auto *motion = new BiDirMotion(si_, &tree_);

        // Sample numSamples_ number of nodes from the free configuration space
        while (nodeCount < numSamples_ && !ptc)
        {
            sampler_->sampleUniform(motion->getState());
            sampleAttempts++;
            if (si_->isValid(motion->getState()))
            {  // collision checking
                ++nodeCount;
                nn->add(motion);
                motion = new BiDirMotion(si_, &tree_);
            }
        }
        si_->freeState(motion->getState());
        delete motion;

        // 95% confidence limit for an upper bound for the true free space volume
        freeSpaceVolume_ =
                boost::math::binomial_distribution<>::find_upper_bound_on_p(sampleAttempts, nodeCount, 0.05) *
                si_->getStateSpace()->getMeasure();
        std::cout << "sample end" << std::endl;//test code
    }
    else
    {
        base::InformedSamplerPtr infSampler_; // informed sampler with single-point sampling

        infSampler_.reset();  // informed sampler reset
        if (static_cast<bool>(opt_) == true)
        {
            if (opt_->hasCostToGoHeuristic() == false)
            {
                OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
            }
        }
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases

        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, 100u);
        unsigned int nodeCount = 0;
        unsigned int sampleAttempts = 0;
        auto *motion = new BiDirMotion(si_, &tree_);

        // Sample numSamples_ number of nodes from the free configuration space
        while (nodeCount < numSamples_ && !ptc)
        {
            infSampler_->sampleUniform(motion->getState(), lastCost);
            sampleAttempts++;

            bool collision_free = si_->isValid(motion->getState());

            if (collision_free)
            {
                nodeCount++;
                nn->add(motion);
                motion = new BiDirMotion(si_, &tree_);
            }  // If collision free
        }      // While nodeCount < numSamples
        si_->freeState(motion->getState());
        delete motion;

        // 95% confidence limit for an upper bound for the true free space volume

        freeSpaceVolume_ = boost::math::binomial_distribution<>::find_upper_bound_on_p(sampleAttempts, nodeCount, 0.05) *
                si_->getStateSpace()->getMeasure();
    }
}

double IHBFMT::calculateUnitBallVolume(const unsigned int dimension) const
{
    if (dimension == 0)
        return 1.0;
    if (dimension == 1)
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension * calculateUnitBallVolume(dimension - 2);
}

double IHBFMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0 / (double)dimension;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) *
            std::pow(log((double)n) / (double)n, a);
}

void IHBFMT::initializeProblem(base::GoalSampleableRegion *&goal_s)
{
    checkValidity();
    if (!sampler_)
    {
        sampler_ = si_->allocStateSampler();
    }
    goal_s = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
}

base::PlannerStatus IHBFMT::solve(const base::PlannerTerminationCondition &ptc)
{
    base::GoalSampleableRegion *goal_s;
    initializeProblem(goal_s);
    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    useFwdTree();

    // Add start states to Unvisitedfwd and Openfwd
    bool valid_initMotion = false;
    //            BiDirMotion *initMotion;
    while (const base::State *st = pis_.nextStart())
    {
        initMotion = new BiDirMotion(si_, &tree_);
        si_->copyState(initMotion->getState(), st);

        initMotion->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
        nn_->add(initMotion);  // S <-- {x_init}
        if (si_->isValid(initMotion->getState()))
        {
            // Take the first valid initial state as the forward tree root
            Open_elements[FWD][initMotion] = Open_[FWD].insert(initMotion);
            initMotion->currentSet_[FWD] = BiDirMotion::SET_OPEN;
            initMotion->cost_[FWD] = opt_->initialCost(initMotion->getState());
            valid_initMotion = true;
            heurGoalState_[1] = initMotion->getState();
        }
    }

    if ((initMotion == nullptr) || !valid_initMotion)
    {
        OMPL_ERROR("Start state undefined or invalid.");
        return base::PlannerStatus::INVALID_START;
    }


    // Add goal states to Unvisitedrev and Openrev
    bool valid_goalMotion = false;
    //            BiDirMotion *goalMotion;
    while (const base::State *st = pis_.nextGoal())
    {
        goalMotion = new BiDirMotion(si_, &tree_);
        si_->copyState(goalMotion->getState(), st);

        goalState_=goalMotion->getState();

        goalMotion->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;
        nn_->add(goalMotion);  // S <-- {x_goal}
        if (si_->isValid(goalMotion->getState()))
        {
            // Take the first valid goal state as the reverse tree root
            Open_elements[REV][goalMotion] = Open_[REV].insert(goalMotion);
            goalMotion->currentSet_[REV] = BiDirMotion::SET_OPEN;
            goalMotion->cost_[REV] = opt_->terminalCost(goalMotion->getState());
            valid_goalMotion = true;
            heurGoalState_[0] = goalMotion->getState();
        }
    }

    if ((goalMotion == nullptr) || !valid_goalMotion)
    {
        OMPL_ERROR("Goal state undefined or invalid.");
        return base::PlannerStatus::INVALID_GOAL;
    }

    useRevTree();

    // Plan a path
    BiDirMotion *connection_point = nullptr;
    bool earlyFailure = true;

    if (initMotion != nullptr && goalMotion != nullptr)
    {
        earlyFailure = plan(initMotion, goalMotion, connection_point, ptc);
    }
    else
    {
        OMPL_ERROR("Initial/goal state(s) are undefined!");
    }

    if (earlyFailure)
    {
        return base::PlannerStatus(false, false);
        //replan(initMotion, goalMotion, connection_point, ptc);
        //                clear();
    }


    // Save the best path (through z)
    if (!ptc)
    {
        // Set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath_.size() - 1; i >= 0; --i)
        {
            path->append(mpath_[i]->getState());
        }

        static const bool approximate = false;
        static const double cost_difference_from_goal = 0.0;
        pdef_->addSolutionPath(path, approximate, cost_difference_from_goal, getName());

        //                OMPL_DEBUG("Total path cost: %f\n", fwd_cost.value() + rev_cost.value());
        return base::PlannerStatus(true, false);
    }

    // Planner terminated without accomplishing goal
    return base::PlannerStatus(false, false);
}

void IHBFMT::traceSolutionPathThroughTree(BiDirMotion *&connection_point)
{
    base::Cost fwd_cost, rev_cost, connection_cost;

    // Construct the solution path
    useFwdTree();
    BiDirMotionPtrs path_fwd;
    tracePath(connection_point, path_fwd);
    fwd_cost = connection_point->getCost();

    useRevTree();
    BiDirMotionPtrs path_rev;
    tracePath(connection_point, path_rev);
    rev_cost = connection_point->getCost();

    // ASSUMES FROM THIS POINT THAT z = path_fwd[0] = path_rev[0]
    // Remove the first element, z, in the traced reverse path
    // (the same as the first element in the traced forward path)
    if (path_rev.size() > 1)
    {
        connection_cost = base::Cost(rev_cost.value() - path_rev[1]->getCost().value());
        path_rev.erase(path_rev.begin());
    }
    else if (path_fwd.size() > 1)
    {
        connection_cost = base::Cost(fwd_cost.value() - path_fwd[1]->getCost().value());
        path_fwd.erase(path_fwd.begin());
    }
    else
    {
        OMPL_ERROR("Solution path traced incorrectly or otherwise constructed improperly \
                   through forward/reverse trees (both paths are one node in length, each).");
    }

    // Adjust costs/parents in reverse tree nodes as cost/direction from forward tree root
    useFwdTree();
    path_rev[0]->setCost(base::Cost(path_fwd[0]->getCost().value() + connection_cost.value()));
    path_rev[0]->setParent(path_fwd[0]);
    for (unsigned int i = 1; i < path_rev.size(); ++i)
    {
        path_rev[i]->setParent(path_rev[i - 1]);
    }

    mpath_.clear();
    std::reverse(path_rev.begin(), path_rev.end());
    mpath_.reserve(path_fwd.size() + path_rev.size());  // preallocate memory
    mpath_.insert(mpath_.end(), path_rev.begin(), path_rev.end());
    mpath_.insert(mpath_.end(), path_fwd.begin(), path_fwd.end());

    mPathSize_ = mpath_.size();
    OMPL_DEBUG("total path cost: %f\n", fwd_cost.value() + rev_cost.value());

    lastCost = opt_->combineCosts(fwd_cost,rev_cost);

    std::reverse(path_rev.begin(), path_rev.end());
    std::vector<BiDirMotion*>::iterator iter = std::find(path_rev[0]->getParent()->getChildren().begin()
            ,path_rev[0]->getParent()->getChildren().end()
            ,path_rev[0]);
    //vector< int >::iterator iter=std::find(v.begin(),v.end(),num_to_find);
    if(iter != path_rev[0]->getParent()->getChildren().end())
    {   if(path_rev[0]->getParent()!=nullptr)
            path_rev[0]->getParent()->getChildren().erase(iter);
    }
    for (unsigned int i = 1; i < path_rev.size(); ++i)
    {
        std::vector<BiDirMotion*>::iterator iter = std::find(path_rev[i]->getParent()->getChildren().begin()
                                                             ,path_rev[i]->getParent()->getChildren().end()
                                                             ,path_rev[i]);
        if(iter != path_rev[i]->getParent()->getChildren().end())
        {
            std::cout<<"already to remove from parent"<< std::endl;
            if(path_rev[i]->getParent()!=nullptr)
                path_rev[i]->getParent()->getChildren().erase(iter);
        }
    }
}


void IHBFMT::expandTreeFromNode(BiDirMotion *&z, BiDirMotion *&connection_point)
{
    // Define Opennew and set it to NULL
    BiDirMotionPtrs Open_new;

    // Define Znear as all unexplored nodes in the neighborhood around z
    BiDirMotionPtrs zNear;
    const BiDirMotionPtrs &zNeighborhood = neighborhoods_[z];

    for (auto i : zNeighborhood)
    {
        if (i->getCurrentSet() == BiDirMotion::SET_UNVISITED)
        {
            zNear.push_back(i);
        }
    }

    // For each node x in Znear
    for (auto x : zNear)
    {
        if (!precomputeNN_)
            saveNeighborhood(nn_, x);  // nearest neighbors

        // Define Xnear as all frontier nodes in the neighborhood around the unexplored node x
        BiDirMotionPtrs xNear;
        const BiDirMotionPtrs &xNeighborhood = neighborhoods_[x];
        for (auto j : xNeighborhood)
        {
            if (j->getCurrentSet() == BiDirMotion::SET_OPEN)
            {
                xNear.push_back(j);
            }
        }
        // Find the node in Xnear with minimum cost-to-come in the current tree
        BiDirMotion *xMin = nullptr;
        double cMin = std::numeric_limits<double>::infinity();
        for (auto &j : xNear)
        {
            // check if node costs are smaller than minimum
            double cNew = j->getCost().value() + distanceFunction(j, x);

            if (cNew < cMin)
            {
                xMin = j;
                cMin = cNew;
            }
        }

        // xMin was found
        if (xMin != nullptr)
        {
            bool collision_free = false;
            if (cacheCC_)
            {
                if (!xMin->alreadyCC(x))
                {
                    collision_free = si_->checkMotion(xMin->getState(), x->getState());
                    ++collisionChecks_;
                    // Due to FMT3* design, it is only necessary to save unsuccesful
                    // connection attemps because of collision
                    if (!collision_free)
                        xMin->addCC(x);
                }
            }
            else
            {
                ++collisionChecks_;
                collision_free = si_->checkMotion(xMin->getState(), x->getState());
            }

            if (collision_free)
            {  // motion between yMin and x is obstacle free
                // add edge from xMin to x
                x->setParent(xMin);
                x->setCost(base::Cost(cMin));
                xMin->getChildren().push_back(x);

                if (heuristics_)
                    x->setHeuristicCost(opt_->motionCostHeuristic(x->getState(), heurGoalState_[tree_]));

                // check if new node x is in the other tree; if so, save result
                if (x->getOtherSet() != BiDirMotion::SET_UNVISITED)
                {
                    if (connection_point == nullptr)
                    {
                        connection_point = x;

                        connect_motion_=connection_point;
                        std::cout << "connect_motion has been first found:  " <<
                                     connection_point->cost_[FWD].value()+connection_point->cost_[REV].value() << std::endl;//test code



                        if (termination_ == FEASIBILITY)
                        {
                            break;
                        }
                    }
                    else
                    {
                        if ((connection_point->cost_[FWD].value() + connection_point->cost_[REV].value()) >
                                (x->cost_[FWD].value() + x->cost_[REV].value()))
                        {
                            connection_point = x;

                            connect_motion_=connection_point;
                            connectchange_ = true;

                        }
                    }
                }

                Open_new.push_back(x);                      // add x to Open_new
                x->setCurrentSet(BiDirMotion::SET_CLOSED);  // remove x from Unvisited
            }
        }
    }  // End "for x in Znear"

    // Remove motion z from binary heap and map
    BiDirMotionBinHeap::Element *zElement = Open_elements[tree_][z];
    Open_[tree_].remove(zElement);
    Open_elements[tree_].erase(z);
    z->setCurrentSet(BiDirMotion::SET_CLOSED);

    // add nodes in Open_new to Open
    for (auto &i : Open_new)
    {
        Open_elements[tree_][i] = Open_[tree_].insert(i);
        i->setCurrentSet(BiDirMotion::SET_OPEN);
    }
}

void IHBFMT::improvedExpandTreeFromNode(BiDirMotion *&z, BiDirMotion *&connection_point)
{
    // Define Opennew and set it to NULL
    BiDirMotionPtrs Open_new;

    // Define Znear as all unexplored nodes in the neighborhood around z
    BiDirMotionPtrs zNear;
    const BiDirMotionPtrs &zNeighborhood = neighborhoods_[z];

    for (auto i : zNeighborhood)
    {
        if (i->getCurrentSet() == BiDirMotion::SET_UNVISITED)
        {
            zNear.push_back(i);
        }
    }

    // For each node x in Znear
    for (auto x : zNear)
    {
        if (!precomputeNN_)
            saveNeighborhood(nn_, x);  // nearest neighbors

        // Define Xnear as all frontier nodes in the neighborhood around the unexplored node x
        BiDirMotionPtrs xNear;
        const BiDirMotionPtrs &xNeighborhood = neighborhoods_[x];
        for (auto j : xNeighborhood)
        {
            if (j->getCurrentSet() == BiDirMotion::SET_OPEN)
            {
                xNear.push_back(j);
            }
        }
        // Find the node in Xnear with minimum cost-to-come in the current tree
        BiDirMotion *xMin = nullptr;
        double cMin = std::numeric_limits<double>::infinity();
        for (auto &j : xNear)
        {
            // check if node costs are smaller than minimum
            double cNew = j->getCost().value() + distanceFunction(j, x);


            if (j->getCost().value() < 0)
            {
                std::cerr << "[DEBUG] Parent motion has invalid negative cost: " << j->getCost().value()
                          << " at address " << j << std::endl;
                continue;
            }

            if (cNew < cMin)
            {
                xMin = j;
                cMin = cNew;
            }
        }

        // xMin was found
        if (xMin != nullptr)
        {
            bool collision_free = false;
            if (cacheCC_)
            {
                if (!xMin->alreadyCC(x))
                {
                    collision_free = si_->checkMotion(xMin->getState(), x->getState());
                    ++collisionChecks_;
                    // Due to FMT3* design, it is only necessary to save unsuccesful
                    // connection attemps because of collision
                    if (!collision_free)
                        xMin->addCC(x);
                }
            }
            else
            {
                ++collisionChecks_;
                collision_free = si_->checkMotion(xMin->getState(), x->getState());
            }

            if (collision_free)
            {  // motion between yMin and x is obstacle free
                // add edge from xMin to x
                x->setParent(xMin);
                x->setCost(base::Cost(cMin));
                xMin->getChildren().push_back(x);

                if (heuristics_)
                    x->setHeuristicCost(opt_->motionCostHeuristic(x->getState(), heurGoalState_[tree_]));

                // check if new node x is in the other tree; if so, save result
                if (x->getOtherSet() != BiDirMotion::SET_UNVISITED)
                {


                    if (x->parent_[FWD] != nullptr && x->parent_[REV] != nullptr)
                    {

                        if (connection_point == nullptr)
                        {
                            connection_point = x;

                            connect_motion_=connection_point;
                            std::cout << "connect_motion has been first found:  " <<
                                         connection_point->cost_[FWD].value()+connection_point->cost_[REV].value() << std::endl;//test code

                            if (termination_ == FEASIBILITY)
                            {
                                break;
                            }
                        }
                        else
                        {
                            if ((connection_point->cost_[FWD].value() + connection_point->cost_[REV].value()) >
                                    (x->cost_[FWD].value() + x->cost_[REV].value()))
                            {

                                connection_point = x;

                                connect_motion_=connection_point;
                                connectchange_ = true;

                                lastCost = opt_->combineCosts(connection_point->cost_[FWD],connection_point->cost_[REV]);
                            }
                        }
                    }
                }
                Open_new.push_back(x);                      // add x to Open_new
                x->setCurrentSet(BiDirMotion::SET_CLOSED);  // remove x from Unvisited
            }
        }
        const unsigned int xNeighborhoodSize = xNeighborhood.size();

        if((firstSuccessful_) && (x->getCurrentSet() == BiDirMotion::SET_CLOSED))
            //if (x->getSetType() == Motion::SET_CLOSED) // 优化解
        {
            std::vector<BiDirMotion *> hNear;
            hNear.reserve(xNeighborhoodSize);
            std::vector<base::Cost> costs;
            std::vector<base::Cost> incCosts;
            std::vector<std::size_t> sortedCostIndices;
            CostIndexCompare compareFn(costs, *opt_);

            for (unsigned int i = 0; i < xNeighborhoodSize; ++i)
            {
                if ( (xNeighborhood[i]->getCurrentSet() == BiDirMotion::SET_OPEN)
                     && ( xNeighborhood[i]->getParent() != x->getParent() ) )
                {
                    hNear.push_back(xNeighborhood[i]);
                }
            }
            if (costs.size() < hNear.size())
            {
                costs.resize(hNear.size());
                incCosts.resize(hNear.size());
                sortedCostIndices.resize(hNear.size());
            }
            for (unsigned int i = 0; i < hNear.size(); ++i)
            {
                incCosts[i] = opt_->motionCost(x->getState(), hNear[i]->getState());
                costs[i] = opt_->combineCosts(x->getCost(), incCosts[i]);
            }
            for (std::size_t i = 0; i < hNear.size(); ++i)
            {
                sortedCostIndices[i] = i;
            }
            std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + hNear.size(), compareFn);
            for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                 i != sortedCostIndices.begin() + hNear.size(); ++i)
            {
                if (opt_->isCostBetterThan(costs[*i], hNear[*i]->getCost()))
                {
                    BiDirMotion *hhNear = hNear[*i];
                    bool collision_free = false;
                    if (cacheCC_)
                    {
                        if (!x->alreadyCC(hhNear))
                        {
                            collision_free = si_->checkMotion(hhNear->getState(), x->getState());
                            ++collisionChecks_;
                            if (!collision_free)
                                x->addCC(hhNear);
                        }
                    }
                    else
                    {
                        ++collisionChecks_;
                        collision_free = si_->checkMotion(hhNear->getState(), x->getState());
                    }

                    if (collision_free)
                    {
                        std::vector<BiDirMotion*>::iterator iter = std::find(hhNear->getParent()->getChildren().begin()
                                                                             ,hhNear->getParent()->getChildren().end()
                                                                             ,hhNear);
                        if(iter != hhNear->getParent()->getChildren().end())
                        {
                            hhNear->getParent()->getChildren().erase(iter);
                        }
                        hhNear->setParent(x);
                        hhNear->setCost(costs[*i]);
                        x->getChildren().push_back(hhNear);
//                        updateChildCosts(hhNear);

                        std::unordered_set<BiDirMotion*> visited;
                        updateChildCosts(hhNear, visited);

                    }
                }
            }
        }

    }  // End "for x in Znear"

    // Remove motion z from binary heap and map
    BiDirMotionBinHeap::Element *zElement = Open_elements[tree_][z];

    auto it = Open_elements[tree_].find(z);
    if (it != Open_elements[tree_].end() && it->second != nullptr)
    {
        Open_[tree_].remove(it->second);
        Open_elements[tree_].erase(it);
    }
    else
    {
        OMPL_WARN("Attempted to remove a motion not in Open_elements or with null element pointer.");
    }

    z->setCurrentSet(BiDirMotion::SET_CLOSED);

    // add nodes in Open_new to Open
    for (auto &i : Open_new)
    {
        Open_elements[tree_][i] = Open_[tree_].insert(i);
        i->setCurrentSet(BiDirMotion::SET_OPEN);
    }
}

bool IHBFMT::plan(BiDirMotion *x_init, BiDirMotion *x_goal, BiDirMotion *&connection_point,
                const base::PlannerTerminationCondition &ptc)
{
    // Expand the trees until reaching the termination condition
    bool earlyFailure = false;
    bool success = false;
    bool improvesucess = false;
    int i = 1;
    int in = 1;
    int improveN = numSamples_*2*in;
    while (!firstSuccessful_)
    {
        int firstN = numSamples_*i;
        // Sample N free states in configuration state_
        if (!sampler_)
            sampler_ = si_->allocStateSampler();
        else
        {
            sampler_.reset();
            sampler_ = si_->allocStateSampler();
        }
        setNumSamples(firstN);
        sampleFree(nn_, ptc);  // S <-- SAMPLEFREE(N)
        OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                    nn_->size());

        // Calculate the nearest neighbor search radius
        if (nearestK_)
        {
            NNk_ = std::ceil(std::pow(2.0 * radiusMultiplier_, (double)si_->getStateDimension()) *
                             (boost::math::constants::e<double>() / (double)si_->getStateDimension()) *
                             log((double)nn_->size()));
            OMPL_DEBUG("Using nearest-neighbors k of %d", NNk_);
        }
        else
        {
            NNr_ = calculateRadius(si_->getStateDimension(), nn_->size());
            OMPL_DEBUG("Using radius of %f", NNr_);
        }

        // If pre-computation, find neighborhoods for all N sample nodes plus initial
        // and goal state(s).  Otherwise compute the neighborhoods of the initial and
        // goal states separately and compute the others as needed.
        BiDirMotionPtrs sampleNodes;
        nn_->list(sampleNodes);
        if (precomputeNN_)
        {
            for (auto &sampleNode : sampleNodes)
            {
                saveNeighborhood(nn_, sampleNode);  // nearest neighbors
            }
        }
        else
        {
            saveNeighborhood(nn_, x_init);  // nearest neighbors
            saveNeighborhood(nn_, x_goal);  // nearest neighbors
        }

        // Copy nodes in the sample set to Unvisitedfwd.  Overwrite the label of the initial
        // node with set Open for the forward tree, since it starts in set Openfwd.
        useFwdTree();
        x_init->setCurrentSet(BiDirMotion::SET_OPEN);

        // Copy nodes in the sample set to Unvisitedrev.  Overwrite the label of the goal
        // node with set Open for the reverse tree, since it starts in set Openrev.
        useRevTree();
        x_goal->setCurrentSet(BiDirMotion::SET_OPEN);


        useFwdTree();
        BiDirMotion *z = x_init;

        std::cout << "the " << i << " time planning begin " << std::endl;
        while (!success)
        {
            expandTreeFromNode(z, connection_point);

            drawPathe();

            // Check if the algorithm should terminate.  Possibly redefines connection_point.
            if (termination(z, connection_point, ptc))
            {
                success = true;
                successful_= true;
                firstSuccessful_ = true;
                traceSolutionPathThroughTree(connection_point);

                drawPathe();
                swapTrees();
            }

            else
            {
                if (Open_[tree_].empty())  // If this heap is empty...
                {
                    if (!extendedFMT_)  // ... eFMT not enabled...
                    {
                        OMPL_INFORM("Open is empty before path was found --> no feasible path exists");
                        Planner::clear();
                        if (nn_)
                            nn_->clear();
                        Open_[FWD].clear();
                        Open_[REV].clear();
                        Open_elements[FWD].clear();
                        Open_elements[REV].clear();
                        neighborhoods_.clear();
                        collisionChecks_ = 0;

                        x_init->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
                        nn_->add(x_init);  // S <-- {x_init}
                        if (si_->isValid(x_init->getState()))
                        {
                            // Take the first valid initial state as the forward tree root
                            Open_elements[FWD][x_init] = Open_[FWD].insert(x_init);
                            x_init->currentSet_[FWD] = BiDirMotion::SET_OPEN;
                            x_init->cost_[FWD] = opt_->initialCost(x_init->getState());
                            heurGoalState_[1] = x_init->getState();
                        }


                        x_goal->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;
                        nn_->add(x_goal);  // S <-- {x_goal}
                        if (si_->isValid(x_goal->getState()))
                        {
                            // Take the first valid goal state as the reverse tree root
                            Open_elements[REV][x_goal] = Open_[REV].insert(x_goal);
                            x_goal->currentSet_[REV] = BiDirMotion::SET_OPEN;
                            x_goal->cost_[REV] = opt_->terminalCost(x_goal->getState());
                            heurGoalState_[0] = x_goal->getState();
                        }
                        break;
                    }
                    else
                        insertNewSampleInOpen(ptc);
                    drawPathe();
                }


                if (!ptc)
                    chooseTreeAndExpansionNode(z);
                else
                    return true;                // This function will be always reached with at least one state in one heap. However, if ptc terminates, we should skip this.

            }
        }
        i = i+1;
        tempCost = lastCost;
    }



    while(!ptc)
    {

        std::cout<< "improving planning begin " << std:: endl;
        std::cout<< "cost threshold: " <<
                 costthrold.value()<< std:: endl;
        Open_[FWD].clear();
        Open_[REV].clear();
        Open_elements[FWD].clear();
        Open_elements[REV].clear();
        neighborhoods_.clear();
        BiDirMotion *z;
        connect_motion_ = nullptr;

        int ii = 1;
        std::vector<BiDirMotion *> allMotions;
        nn_->list(allMotions);
        nn_->clear();
        for (BiDirMotion *everyMotion : allMotions)
        {
            base::Cost solutionHeuristic;
            if (ii != 1)
            {
                base::Cost costToCome;

                // Start with infinite cost
                costToCome = opt_->infiniteCost();

                // Find the min from each start
                costToCome = opt_->betterCost(
                            costToCome, opt_->motionCost(initMotion->getState(),
                                                         everyMotion->getState()));  // lower-bounding cost from the start to the state

                const base::Cost costToGo =
                        opt_->costToGo(everyMotion->getState(), pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
                solutionHeuristic = opt_->combineCosts(costToCome, costToGo);            // add the two costs
            }

            if (ii == 1)
            {
                Open_elements[FWD][everyMotion] = Open_[FWD].insert(everyMotion);
                everyMotion->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
                everyMotion->currentSet_[FWD] = BiDirMotion::SET_OPEN;
                nn_->add(everyMotion);

            }
            else if ((ii !=1) && (everyMotion->getParent() != nullptr)
                     && (opt_->isCostBetterThan(solutionHeuristic,lastCost)))
            {
                Open_elements[FWD][everyMotion] = Open_[FWD].insert(everyMotion);
                everyMotion->currentSet_[FWD] = BiDirMotion::SET_OPEN;
                everyMotion->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
                nn_->add(everyMotion);
            }
            else if ((ii !=1) && (everyMotion->getParent() == nullptr)
                     && (opt_->isCostBetterThan(solutionHeuristic,lastCost)))
            {
                useRevTree();
                if(everyMotion->getParent() != nullptr)
                {
                    Open_elements[REV][everyMotion] = Open_[REV].insert(everyMotion);
                    everyMotion->currentSet_[REV] = BiDirMotion::SET_OPEN;
                    everyMotion->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;
                    nn_->add(everyMotion);
                }
                else
                {
                    everyMotion->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;
                    everyMotion->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
                    nn_->add(everyMotion);
                }
                swapTrees();
            }
            ii = ii + 1;
        }

        setNumSamples(improveN);

        if (!sampler_)
            sampler_ = si_->allocStateSampler();
        else
        {
            sampler_.reset();
            sampler_ = si_->allocStateSampler();
        }


        sampleFree(nn_,ptc);
        OMPL_INFORM("%s: Improving planning with %u states already in datastructure", getName().c_str(), nn_->size());
        if (nearestK_)
        {
            NNk_ = std::ceil(std::pow(2.0 * radiusMultiplier_, (double)si_->getStateDimension()) *
                             (boost::math::constants::e<double>() / (double)si_->getStateDimension()) *
                             log((double)nn_->size()));
            OMPL_DEBUG("Using nearest-neighbors k of %d", NNk_);
        }
        else
        {
            NNr_ = calculateRadius(si_->getStateDimension(), nn_->size());
            OMPL_DEBUG("Using radius of %f", NNr_);
        }

        BiDirMotionPtrs sampleNodes;
        nn_->list(sampleNodes);
        // to do This precomputation is useful only if the same planner is used many times.
        // otherwise is probably a waste of time. Do a real precomputation before calling solve().
        if (precomputeNN_)
        {
            for (auto &sampleNode : sampleNodes)
            {
                saveNeighborhood(nn_, sampleNode);  // nearest neighbors
            }
        }
        else
        {
            saveNeighborhood(nn_, x_init);  // nearest neighbors
            saveNeighborhood(nn_, x_goal);  // nearest neighbors
        }
        useFwdTree();
        x_init->setCurrentSet(BiDirMotion::SET_OPEN);
        x_init->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
        x_init->currentSet_[FWD] = BiDirMotion::SET_OPEN;
        x_init->cost_[FWD] = opt_->initialCost(x_init->getState());
        heurGoalState_[1] = x_init->getState();

        useRevTree();
        x_goal->setCurrentSet(BiDirMotion::SET_OPEN);
        x_goal->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;


        Open_elements[REV][x_goal] = Open_[REV].insert(x_goal);
        x_goal->currentSet_[REV] = BiDirMotion::SET_OPEN;
        x_goal->cost_[REV] = opt_->terminalCost(x_goal->getState());
        heurGoalState_[0] = x_goal->getState();

        useFwdTree();
        z = x_init;

        auto startTime = std::chrono::steady_clock::now();

        while (!ptc)
        {

            improvedExpandTreeFromNode(z, connection_point);

            drawPathe();

            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();


            // Check if the algorithm should terminate.  Possibly redefines connection_point.
            if ((tempCost.value() - lastCost.value()) >= 0.02 * tempCost.value() || opt_->isSatisfied(lastCost))
            {
                tempCost = lastCost ;

                success = true;
                successful_= true;
                firstSuccessful_ = true;
                traceSolutionPathThroughTree(connection_point);
                drawPathe();


                break;

            }

            if( tempCost.value() == lastCost.value() && elapsed >= 30 || elapsed >= 60)
            {
                startTime = std::chrono::steady_clock::now();
                break;

            }

            if (!ptc)
                chooseTreeAndExpansionNode(z);
            else
                return true;
            // This function will be always reached with at least one state in one heap.
            // However, if ptc terminates, we should skip this.



        }

        if (ptc || opt_->isSatisfied(lastCost))
            break;


    }
    earlyFailure = false;
    return earlyFailure;
}

void ompl::geometric::IHBFMT::updateChildCosts(BiDirMotion *m, std::unordered_set<BiDirMotion*> &visited)
{
    if (!m || visited.count(m))
        return;

    visited.insert(m);

    for (unsigned int i = 0; i < m->getChildren().size(); ++i)
    {
        BiDirMotion *child = m->getChildren()[i];

        if (!child)
            continue;

        base::Cost incCost = opt_->motionCost(m->getState(), child->getState());
        child->setCost(opt_->combineCosts(m->getCost(), incCost));

        updateChildCosts(child, visited);
    }
}


void IHBFMT::drawPathe()
{
    static bool winInit = false;
    if (!winInit)
    {
        cv::namedWindow("path", cv::WINDOW_NORMAL);
        cv::resizeWindow("path", 800, 600);
        winInit = true;
    }

    // 1. First, make a copy of the background image .
    cv::Mat frame = envImage_.clone();

    // 2. Draw all the nodes at once.
    BiDirMotionPtrs motions;
    nn_->list(motions);
    for (auto *m : motions)
    {
        auto *s = m->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
        cv::Point p(s->values[0], s->values[1]);

        // Sample
        cv::circle(frame, p, 3, cv::Scalar(0, 128, 255), -1, CV_AA);

        // FWD_tree
        if (m->currentSet_[FWD] != BiDirMotion::SET_UNVISITED && m->parent_[FWD])
        {
            auto *ps = m->parent_[FWD]->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
            cv::Point pp(ps->values[0], ps->values[1]);
            cv::line(frame, pp, p, cv::Scalar(0,255,0), 1, CV_AA);
        }

        // REV_tree
        if (m->currentSet_[REV] != BiDirMotion::SET_UNVISITED && m->parent_[REV])
        {
            auto *ps = m->parent_[REV]->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
            cv::Point pp(ps->values[0], ps->values[1]);
            cv::line(frame, pp, p, cv::Scalar(255,0,255), 1,CV_AA);
        }
    }

    // 3. start/end
    {
        auto *ss = motions.front()->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
        cv::circle(frame, cv::Point(ss->values[0], ss->values[1]), 8, cv::Scalar(128,128,0), -1, CV_AA);
        auto *gs = goalState_->as<ompl::base::RealVectorStateSpace::StateType>();
        cv::circle(frame, cv::Point(gs->values[0], gs->values[1]), 8, cv::Scalar(0,255,165), -1, CV_AA);
    }

    // 4. If a solution exists, draw the solution path as a thick purple line
    if (!mpath_.empty())
    {
        for (size_t i = 1; i < mpath_.size(); ++i)
        {
            auto *s0 = mpath_[i-1]->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
            auto *s1 = mpath_[i  ]->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
            cv::line(frame,
                     cv::Point(s0->values[0], s0->values[1]),
                    cv::Point(s1->values[0], s1->values[1]),
                    cv::Scalar(255, 0, 0), 2,CV_AA);
        }
    }

    cv::imshow("path", frame);
    cv::waitKey(1);
}



void IHBFMT::insertNewSampleInOpen(const base::PlannerTerminationCondition &ptc)
{
    // Sample and connect samples to tree only if there is
    // a possibility to connect to unvisited nodes.
    std::vector<BiDirMotion *> nbh;
    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    auto *m = new BiDirMotion(si_, &tree_);
    while (!ptc && Open_[tree_].empty())
    {
        // Get new sample and check whether it is valid.
        sampler_->sampleUniform(m->getState());
        if (!si_->isValid(m->getState()))
            continue;

        // Get neighbours of the new sample.
        std::vector<BiDirMotion *> yNear;
        if (nearestK_)
            nn_->nearestK(m, NNk_, nbh);
        else
            nn_->nearestR(m, NNr_, nbh);

        yNear.reserve(nbh.size());
        for (auto &j : nbh)
        {
            if (j->getCurrentSet() == BiDirMotion::SET_CLOSED)
            {
                if (nearestK_)
                {
                    // Only include neighbors that are mutually k-nearest
                    // Relies on NN datastructure returning k-nearest in sorted order
                    const base::Cost connCost = opt_->motionCost(j->getState(), m->getState());
                    const base::Cost worstCost =
                            opt_->motionCost(neighborhoods_[j].back()->getState(), j->getState());

                    if (opt_->isCostBetterThan(worstCost, connCost))
                        continue;
                    yNear.push_back(j);
                }
                else
                    yNear.push_back(j);
            }
        }

        // Sample again if the new sample does not connect to the tree.
        if (yNear.empty())
            continue;

        // cache for distance computations
        //
        // Our cost caches only increase in size, so they're only
        // resized if they can't fit the current neighborhood
        if (costs.size() < yNear.size())
        {
            costs.resize(yNear.size());
            incCosts.resize(yNear.size());
            sortedCostIndices.resize(yNear.size());
        }

        // Finding the nearest neighbor to connect to
        // By default, neighborhood states are sorted by cost, and collision checking
        // is performed in increasing order of cost
        //
        // calculate all costs and distances
        for (std::size_t i = 0; i < yNear.size(); ++i)
        {
            incCosts[i] = opt_->motionCost(yNear[i]->getState(), m->getState());
            costs[i] = opt_->combineCosts(yNear[i]->getCost(), incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < yNear.size(); ++i)
            sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + yNear.size(), compareFn);

        // collision check until a valid motion is found
        for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
             i != sortedCostIndices.begin() + yNear.size(); ++i)
        {
            ++collisionChecks_;
            if (si_->checkMotion(yNear[*i]->getState(), m->getState()))
            {
                const base::Cost incCost = opt_->motionCost(yNear[*i]->getState(), m->getState());
                m->setParent(yNear[*i]);
                yNear[*i]->getChildren().push_back(m);
                m->setCost(opt_->combineCosts(yNear[*i]->getCost(), incCost));
                m->setHeuristicCost(opt_->motionCostHeuristic(m->getState(), heurGoalState_[tree_]));
                m->setCurrentSet(BiDirMotion::SET_OPEN);
                Open_elements[tree_][m] = Open_[tree_].insert(m);

                nn_->add(m);
                saveNeighborhood(nn_, m);
                updateNeighborhood(m, nbh);

                break;
            }
        }
    }  // While Open_[tree_] empty
}

bool IHBFMT::termination(BiDirMotion *&z, BiDirMotion *&connection_point,
                       const base::PlannerTerminationCondition &ptc)
{
    bool terminate = false;
    if(!firstSuccessful_)
    {
        switch (termination_)
        {
        case FEASIBILITY:
            // Test if a connection point was found during tree expansion
            return (connection_point != nullptr || ptc);

            break;

        case OPTIMALITY:
            // Test if z is in SET_CLOSED (interior) of other tree
            if (ptc)
                terminate = true;
            else if (z->getOtherSet() == BiDirMotion::SET_CLOSED)
                terminate = true;

            break;
        };
        return terminate;
    }
    else
    {
        switch (termination_)
        {
        case FEASIBILITY:
            // Test if a connection point was found during tree expansion
            return (connect_motion_ != nullptr || ptc);
            break;

        case OPTIMALITY:
            // Test if z is in SET_CLOSED (interior) of other tree
            if (ptc)
                terminate = true;
            else if (z->getOtherSet() == BiDirMotion::SET_CLOSED)
            {
                terminate = true;
                std::cout<<"arrive termination"<< std::endl;
            }
            break;
        };
        return terminate;
    }
}

// Choose exploration tree and node z to expand
void IHBFMT::chooseTreeAndExpansionNode(BiDirMotion *&z)
{
    switch (exploration_)
    {
    case SWAP_EVERY_TIME:
        if (Open_[(tree_ + 1) % 2].empty())
            z = Open_[tree_].top()->data;  // Continue expanding the current tree (not empty by exit
        else
        {
            z = Open_[(tree_ + 1) % 2].top()->data;  // Take top of opposite tree heap as new z
            swapTrees();                             // Swap to the opposite tree
        }
        break;

    case CHOOSE_SMALLEST_Z:
        BiDirMotion *z1, *z2;
        if (Open_[(tree_ + 1) % 2].empty())
            z = Open_[tree_].top()->data;  // Continue expanding the current tree (not empty by exit
        else if (Open_[tree_].empty())
        {
            z = Open_[(tree_ + 1) % 2].top()->data;  // Take top of opposite tree heap as new z
            swapTrees();                             // Swap to the opposite tree
        }
        else
        {
            z1 = Open_[tree_].top()->data;
            z2 = Open_[(tree_ + 1) % 2].top()->data;

            if (z1->getCost().value() < z2->getOtherCost().value())
                z = z1;
            else
            {
                z = z2;
                swapTrees();
            }
        }
        break;
    };
}

// Trace a path of nodes along a tree towards the root (forward or reverse)
void IHBFMT::tracePath(BiDirMotion *z, BiDirMotionPtrs &path)
{
    BiDirMotion *solution = z;

    while (solution != nullptr)
    {
        path.push_back(solution);
        solution = solution->getParent();
    }
}

void IHBFMT::swapTrees()
{
    tree_ = (TreeType)((((int)tree_) + 1) % 2);
}

void IHBFMT::updateNeighborhood(BiDirMotion *m, const std::vector<BiDirMotion *> nbh)
{
    // Neighborhoods are only updated if the new motion is within bounds (k nearest or within r).
    for (auto i : nbh)
    {
        // If CLOSED, that neighborhood won't be used again.
        // Else, if neighhboorhod already exists, we have to insert the node in
        // the corresponding place of the neighborhood of the neighbor of m.
        if (i->getCurrentSet() == BiDirMotion::SET_CLOSED)
            continue;

        auto it = neighborhoods_.find(i);
        if (it != neighborhoods_.end())
        {
            if (it->second.empty())
                continue;

            const base::Cost connCost = opt_->motionCost(i->getState(), m->getState());
            const base::Cost worstCost = opt_->motionCost(it->second.back()->getState(), i->getState());

            if (opt_->isCostBetterThan(worstCost, connCost))
                continue;

            // insert the neighbor in the vector in the correct order
            std::vector<BiDirMotion *> &nbhToUpdate = it->second;
            for (std::size_t j = 0; j < nbhToUpdate.size(); ++j)
            {
                // If connection to the new state is better than the current neighbor tested, insert.
                const base::Cost cost = opt_->motionCost(i->getState(), nbhToUpdate[j]->getState());
                if (opt_->isCostBetterThan(connCost, cost))
                {
                    nbhToUpdate.insert(nbhToUpdate.begin() + j, m);
                    break;
                }
            }
        }
    }
}
}  // End "geometric" namespace
}  // End "ompl" namespace


